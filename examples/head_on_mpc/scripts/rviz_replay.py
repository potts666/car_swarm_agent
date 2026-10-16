#!/usr/bin/env python3
"""Replay head-on simulation logs as RViz markers; no controller execution."""
import math
import time

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from recording import Recording

COLORS = [(0.15, .65, 1.), (1., .48, .12)]


class Replay(Node):
    def __init__(self):
        super().__init__('head_on_replay')
        directory = self.declare_parameter('log_dir', '').value
        self.rate = float(self.declare_parameter('playback_rate', 1.).value)
        self.loop = self.declare_parameter('loop', True).value
        if not directory or not math.isfinite(self.rate) or self.rate <= 0:
            raise ValueError('Set log_dir to a case folder and playback_rate to a positive finite value')
        self.recording = Recording(directory)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, '/mpc_replay/markers', qos)
        self.tf = TransformBroadcaster(self)
        self.started = time.monotonic()
        self.create_timer(.05, self.tick)
        self.get_logger().info(f'Replaying {directory}; {self.recording.end-self.recording.start:.1f}s; no optimizer/control publication')
        self.tick()

    def marker(self, namespace, index, kind, color, x=0., y=0., z=0.):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type = namespace, index, kind
        marker.pose.orientation.w = 1.
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(x), float(y), float(z)
        marker.color.r, marker.color.g, marker.color.b = color[:3]
        marker.color.a = color[3] if len(color) > 3 else 1.
        return marker

    def box(self, namespace, index, color, x0, x1, y0, y1, z, height):
        marker = self.marker(namespace, index, Marker.CUBE, color, (x0+x1)/2, (y0+y1)/2, z)
        marker.scale.x, marker.scale.y, marker.scale.z = float(x1-x0), float(y1-y0), float(height)
        return marker

    def line(self, namespace, index, color, points, width, z):
        marker = self.marker(namespace, index, Marker.LINE_STRIP, color)
        marker.scale.x = width
        marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y in points]
        if len(marker.points) < 2:
            marker.action = Marker.DELETE
        return marker

    def tick(self):
        recording = self.recording
        elapsed = (time.monotonic()-self.started)*self.rate
        duration = recording.end-recording.start
        # Hold the final frame for two simulation seconds before looping.
        local = elapsed % (duration+2.) if self.loop else elapsed
        stamp = recording.start+min(local, duration)
        samples = {car: recording.sample(car, stamp) for car in recording.cars}
        anchor = sum(s['x'] for s in samples.values())/len(samples)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'case_view'
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.transform.translation.x = anchor
        transform.transform.translation.y = 1.5
        transform.transform.rotation.w = 1.
        self.tf.sendTransform(transform)
        road, settings = recording.config['road'], recording.config['settings']
        markers = []
        markers.append(self.box('road', 0, (.24,.26,.29), road['xmin'], road['xmax'],
                                road['ymin'], road['ymax'], -.08, .1))
        for i, bay in enumerate(road['bays'], 1):
            markers.append(self.box('road', i, (.2,.38,.26), bay['xmin'], bay['xmax'],
                                    road['ymax'], bay['ymax'], -.08, .1))
        upper = [(road['xmin'], road['ymax'])]
        for bay in sorted(road['bays'], key=lambda b: b['xmin']):
            upper.extend([(bay['xmin'], road['ymax']), (bay['xmin'], bay['ymax']),
                          (bay['xmax'], bay['ymax']), (bay['xmax'], road['ymax'])])
        upper.append((road['xmax'], road['ymax']))
        outline = [(road['xmin'], road['ymin']), (road['xmax'], road['ymin'])]+list(reversed(upper))+[(road['xmin'],road['ymin'])]
        markers.append(self.line('road', 100, (.8,.8,.8), outline, .09, .02))
        for i, car in enumerate(recording.cars):
            s, color = samples[car], COLORS[i % len(COLORS)]
            length, width = settings['length'], settings['width']
            markers.append(self.box(car, 0, (*color,.85), s['x']-length/2, s['x']+length/2,
                                    s['y']-width/2, s['y']+width/2, .4, .7))
            direction = -1 if car == 'B' else 1
            nose = s['x']+direction*(length/2-.35)
            markers.append(self.box(car, 1, (.98,.98,.98), nose-.12, nose+.12,
                                    s['y']-width*.35, s['y']+width*.35, .77, .04))
            label = self.marker(car, 2, Marker.TEXT_VIEW_FACING, color, s['x'], s['y']-1.6, 1.)
            label.scale.z = .6
            label.text = f"{car}  vx={s['vx']:+.2f}  a={s['acceleration']:+.2f}"
            markers.append(label)
            trail = recording.trail(car, stamp)+[(s['x'], s['y'])]
            markers.append(self.line(car, 3, (*color,.85), trail, .1, .06))
            markers.append(self.line(car, 4, (*color,.38), [(s['x'],s['y'])]+recording.plan(car, stamp), .055, .12))
        phase = next(iter(samples.values()))['phase']
        title = self.marker('status', 0, Marker.TEXT_VIEW_FACING, (.95,.95,.95), anchor, 7., 1.)
        title.scale.z = .7
        title.text = f"{recording.config['case']} | t={stamp-recording.start:.1f}s | {phase}"
        if local >= duration:
            title.text += f" | {recording.summary['status']}"
        markers.append(title)
        note = self.marker('status', 1, Marker.TEXT_VIEW_FACING, (.85,.8,.55), anchor, 5.7, 1.)
        note.scale.z = .42
        note.text = 'LOG REPLAY | thick: executed | thin: predicted | prescribed lateral occupancy'
        markers.append(note)
        self.publisher.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = None
    try:
        node = Replay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
