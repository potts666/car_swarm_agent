"""Materialize the domain feedback laws as exact rational ZOH commands."""
from common import *

def waiting(v,delta):
    v,d=F(v),F(delta);assert 0<=v<=F(1,50) and abs(d)<=F(1,100)
    return [(-v/H,F(0)),(F(0),-d/H)]

def entering(v,delta):
    v,d=F(v),F(delta);assert F(49,100)<=v<=F(51,100) and abs(d)<=F(1,1000)
    return ((F(1,2)-v)/H,-d/H)
