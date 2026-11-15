"""Two fixed, proved receiving phases; no extension to unverified future windows."""
from common import *


def receiver(actor,phase):
    assert phase in (819,820),'unregistered receiving phase'
    name='G_in' if phase==819 else 'G_in_shifted';d=read(HERE/'domain_checks.json')['domains'][name];x=F(d.get('center_local_x',-22))*direction(actor)
    domain=dict(center_pose_Q=[[x.numerator,x.denominator],[0,1],[0,1]],position_l1_radius=d['position_l1'],yaw_radius=d['yaw'],center_v='1/2',speed_radius=str((F(d['v'][1])-F(d['v'][0]))/2),center_delta='0',steering_radius=d['delta'][1],feedback_a_ref='0',feedback_omega_ref='0')
    return dict(phase=phase,kind=name,domain=domain,certificate_id=read(HERE/'domain_checks.json')['certificate_id'])


def replacement_events(old,actor,cycle):
    events=copy.deepcopy(old)
    if cycle==788:
        previous=[actor,'G_in'];assert previous in events['819'];events['819'].remove(previous)
        if not events['819']:del events['819']
        events.setdefault('820',[]).append([actor,'G_in_shifted'])
    else:assert cycle==787
    return events
