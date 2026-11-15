"""Immutable completion-policy admission, before any four-car acknowledgement."""
from common import *
from witness import make
_VALIDATED=set()


def validate_strategy(wid):
    if wid in _VALIDATED:return True
    w=get(HERE/'objects',wid);assert sorted(w['order'])==sorted(CARS) and w['model_sha256']==sha(MODEL_PATH) and w['ticks']==2802
    assert w['domain_certificate']==read(HERE/'domain_checks.json')['certificate_id']
    if w.get('optimization_origin'):
        validate_strategy(w['parent_witness'])
        from local_update import verify_update
        verify_update(wid)
    elif w['parent_witness']:
        validate_strategy(w['parent_witness']);old=get(HERE/'objects',w['parent_witness']);expected=make(w['order'],w['parent_witness'],old,w['adoption_rank']);assert w==expected,'unproved completion-policy modification'
    else:assert w==make(w['order']),'uncertified initial completion policy'
    _VALIDATED.add(wid);return True
