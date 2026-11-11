"""Replay every formal release through the final adapter, with stale-message negatives."""
import copy,gzip,json,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[1]/'scripts'))
from run_m2b_jacobi import prepare_resource
from m2b_identity import Identity
from m2b_protocol import Channel,Barrier,canonical,cleared
from m2b_recording import lines
from m2b_jacobi import base

class AdapterTests(unittest.TestCase):
 def test_all_saved_resource_events_and_bad_reports(self):
  counts=dict(runs=0,preparations=0,releases=0,stale_rejected=0,nonowner_rejected=0,missing_rejected=0,wrong_sender_rejected=0)
  for name in ('S-J_1','P-J_1','P-J_2','S-J_2','rename_full','position_full'):
   directory=ROOT/name/'r1';cfg=json.loads((directory/'config.json').read_text());identity=Identity(cfg['identity_mapping']);s=copy.deepcopy(cfg['scene'])
   for key in ('initial','goals'):s[key]={identity.roles[c]:z for c,z in s[key].items()}
   channel=Channel(s,'fifo-ready');agreements={a['cycle']:a for a in lines(directory/'agreements.jsonl')};grouped={}
   # Stored message references need not be expanded; resource/barrier payloads are explicit.
   with gzip.open(directory/'messages.jsonl.gz','rt') as f:
    for line in f:
     m=json.loads(line)
     if m.get('event')=='send' and m['kind'] in ('state','request','release_report'):grouped.setdefault(m['cycle'],[]).append(m)
   for cycle,ms in sorted(grouped.items()):
    for m in ms:
     if m['kind']=='request':channel.request(identity.roles[m['payload']['car']],cycle)
    reports={m['sender']:m['payload'] for m in ms if m['kind']=='state' and m['receiver']=='channel' and not m['payload']['context']['conditional'] and 'coordination_round' not in m['payload']['context']}
    release=next((m for m in ms if m['kind']=='release_report'),None)
    if cycle not in agreements:continue
    if channel.owner is None or cleared(channel.owner,reports[identity.ids[channel.owner]]['z'],s):
     if release:
      counts['releases']+=1
      for kind in ('stale','nonowner','missing','wrong_sender'):
       trial=copy.deepcopy(channel);bad=copy.deepcopy(release);before=copy.deepcopy(vars(trial))
       other=next(c for c in identity.cars if c!=release['sender'])
       if kind=='stale':bad['payload']['authorization_version']-=1
       if kind=='nonowner':bad['sender']=other;bad['payload']['car']=other
       if kind=='missing':bad=None
       if kind=='wrong_sender':bad['sender']=other
       with self.assertRaises(ValueError):prepare_resource(trial,reports,identity,cycle,bad)
       self.assertEqual(vars(trial),before);counts[kind+'_rejected']+=1
     ctx=prepare_resource(channel,reports,identity,cycle,release);counts['preparations']+=1
     saved=base(agreements[cycle]['packet']['token']['context']);self.assertEqual(ctx,saved)
     if ctx['conditional']:
      b=Barrier(identity.context(ctx,False));pair={identity.roles[c]:pid for c,pid in agreements[cycle]['plans'].items()}
      for c,pid in pair.items():b.publish(dict(car=c,context=b.context,kind='final',id=pid))
      for c in pair:b.accept(dict(car=c,context=b.context,plans=pair,accepted=True))
      effective=identity.context(channel.commit(b));self.assertEqual(effective,agreements[cycle]['packet']['effective'])
   counts['runs']+=1
  (ROOT/'release_adapter_replay.json').write_text(json.dumps(dict(passed=True,counts=counts,normal_resource_contexts_equal=True,negative_state_mutation=False),indent=2)+'\n')

if __name__=='__main__':unittest.main()
