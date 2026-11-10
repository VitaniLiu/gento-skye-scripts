"""Robot joint targets only: independent bounded holds, never measured joint fallback."""
from pathlib import Path
import numpy as np
from processing.schema78 import previous

class CommandTargets:
    def __init__(self, specification):
        self.spec=specification
        self.streams={g['name']:{'times':[],'items':[]} for g in specification['groups']}
    def push(self,record):
        for g in self.spec['groups']:
            if record['topic']!=g['topic']:continue
            if record['type']!=g['message_type']:raise ValueError(f"{Path(__file__).resolve()}:CommandTargets.push: wrong type for {record['topic']}")
            values=np.asarray(record['message']['positions'],dtype=float)
            if values.shape!=(g['source_length'],) or not np.isfinite(values).all():raise ValueError(f"{Path(__file__).resolve()}:CommandTargets.push: invalid positions for {record['topic']}, shape={values.shape}")
            if len(g['joint_names'])!=len(g['source_indices']):raise ValueError('command group joint/index mismatch')
            data=self.streams[g['name']];t=record['bag_ns']
            if data['times'] and t<=data['times'][-1]:raise ValueError(f"{Path(__file__).resolve()}:CommandTargets.push: duplicate/backward receive timestamp, topic={record['topic']}, time={t}")
            data['times'].append(t);data['items'].append({'values':values[g['source_indices']], 'source_ns':t,'header_ns':record['header_ns'] if record['header_ns'] is not None else -1})
    def at(self,t,fk):
        names=[];positions=[];received=[];headers=[]
        for g in self.spec['groups']:
            item=previous(self.streams[g['name']],t,g['max_age_ns'])
            names.extend(g['joint_names']);positions.extend(item['values'])
            received.append(item['source_ns']);headers.append(item['header_ns'])
        if len(names)!=len(set(names)):raise ValueError('duplicate command joint names')
        received=np.asarray(received,np.int64);headers=np.asarray(headers,np.int64)
        return dict(poses=fk.poses78(names,positions),source_ns=int(received.max()),header_ns=int(headers[0]) if np.all(headers==headers[0]) else -1,receive_times=received,header_times=headers,ages=t-received)
