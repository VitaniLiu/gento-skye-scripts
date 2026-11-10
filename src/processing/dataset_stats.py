"""Stable LeRobot-compatible stats; never changes installed package files."""
import numpy as np

def stable_episode_stats(episode_data,features):
    from lerobot.datasets.compute_stats import sample_images
    result={}
    for key,data in episode_data.items():
        dtype=features[key]['dtype']
        if dtype=='string':continue
        if dtype in ('image','video'):
            array=sample_images(data).astype(np.float64)/255.0
            axis=(0,2,3);count=len(array);shape=(3,1,1)
            stats={name:np.asarray(fn(array,axis=axis)).reshape(shape) for name,fn in [('min',np.min),('max',np.max),('mean',np.mean),('std',np.std)]}
            for q in (.01,.1,.5,.9,.99):stats[f'q{int(q*100):02d}']=np.quantile(array,q,axis=axis).reshape(shape)
        else:
            array=np.asarray(data);count=len(array)
            if array.ndim==1:array=array[:,None]
            origin=array[0].astype(np.longdouble);centered=(array.astype(np.longdouble)-origin).astype(np.float64)
            stats={'min':array.min(0),'max':array.max(0),'mean':np.asarray(centered.mean(0).astype(np.longdouble)+origin,dtype=np.float64),'std':centered.std(0)}
            for q in (.01,.1,.5,.9,.99):stats[f'q{int(q*100):02d}']=np.asarray(np.quantile(centered,q,axis=0).astype(np.longdouble)+origin,dtype=np.float64)
        stats['count']=np.array([count],dtype=np.int64);result[key]=stats
    return result
