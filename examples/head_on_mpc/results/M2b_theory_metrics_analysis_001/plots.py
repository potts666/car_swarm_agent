"""Standalone figures of observations; no invented theory/robustness curves."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analysis_io import stream
from analyze import OUT,load


if __name__=='__main__':
    figdir=OUT/'figures';figdir.mkdir(exist_ok=True)
    cases={name:list(stream(OUT/'cycle_metrics'/(name.replace('/','__')+'.jsonl.gz'))) for name in ['C-FIFO/r1','L-FIFO/r1']}
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for ax,(name,rows) in zip(axes,cases.items()):
        t=[r['elapsed_s'] for r in rows]
        for layer in ['actual','final','seed']:ax.plot(t,[r['safety'].get(layer,{}).get('gap_margin_m',np.nan) for r in rows],label=layer,lw=1)
        ax.axhline(0,color='black',ls='--',lw=.7);ax.set_ylabel('Clearance margin [m]');ax.set_title(name);ax.legend(ncol=3)
    axes[-1].set_xlabel('Elapsed simulation time [s]');fig.tight_layout();fig.savefig(figdir/'sampled_safety_margins.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(4,1,figsize=(11,9),sharex=True)
    units=['position_m','heading_rad','speed_m_s','steering_rad'];labels=['Position [m]','Wrapped yaw [rad]','Speed [m/s]','Steering [rad]']
    for name,rows in cases.items():
        for ax,unit,label in zip(axes,units,labels):
            ax.plot([r['elapsed_s'] for r in rows],[max([c['successive_final']['maximum'][unit] for c in r['cars'].values() if 'successive_final' in c],default=np.nan) for r in rows],label=name,lw=1)
            ax.set_ylabel(label)
    axes[0].legend();axes[-1].set_xlabel('Elapsed simulation time [s]');fig.suptitle('Observed reoptimization change over common absolute-time window (not a guarantee bound)');fig.tight_layout();fig.savefig(figdir/'aligned_prediction_changes.png',dpi=160);plt.close(fig)
    rows=cases['L-FIFO/r1'];modes=sorted({c['mode'] for r in rows for c in r['cars'].values()});colors=plt.get_cmap('tab10')
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True,gridspec_kw={'height_ratios':[3,1]})
    cars=['A1','B1','A2','B2']
    for j,c in enumerate(cars):
        for mode in modes:
            segments=[(r['elapsed_s'],.2) for r in rows if r['cars'].get(c,{}).get('mode')==mode]
            axes[0].broken_barh(segments,(j-.35,.7),facecolors=colors(modes.index(mode)),label=mode if j==0 else None)
    axes[0].set_yticks(range(4),cars);axes[0].legend(ncol=4,fontsize=8);axes[0].set_title('L-FIFO local modes and effective/conditional owner used for accepted plans')
    owner={None:0,**{c:i+1 for i,c in enumerate(cars)}}
    axes[1].step([r['elapsed_s'] for r in rows],[owner[r['context']['owner']] for r in rows],where='post');axes[1].set_yticks(range(5),['None']+cars);axes[1].set_xlabel('Elapsed simulation time [s]')
    fig.tight_layout();fig.savefig(figdir/'modes_and_owner.png',dpi=160);plt.close(fig)
    print('Three standalone observed-data figures generated.')
