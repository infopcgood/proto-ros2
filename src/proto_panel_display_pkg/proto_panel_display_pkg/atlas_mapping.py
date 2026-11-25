"""Explicit LED order and anatomical side mapping, independent of ROS/driver."""
import json
from pathlib import Path
import numpy as np


class AtlasMap:
    def __init__(self, configuration):
        if configuration.get('schema')!=1:raise ValueError('Unsupported panel-map schema')
        def integer(name):
            value=configuration[name]
            if isinstance(value,bool) or not isinstance(value,int) or value<=0:
                raise ValueError(name+' must be a positive integer')
            return value
        self.width=integer('side_width');self.height=integer('side_height')
        self.leds_per_strip=integer('leds_per_strip');self.num_strips=integer('num_strips')
        pixels=configuration['pixels'];count=self.leds_per_strip*self.num_strips
        if len(pixels)!=count:raise ValueError('Map must have exactly one entry per driver LED slot')
        self.x=np.zeros(count,dtype=np.intp);self.y=np.zeros(count,dtype=np.intp)
        self.valid=np.zeros(count,dtype=bool)
        for i,entry in enumerate(pixels):
            if entry is None:continue
            if not isinstance(entry,(list,tuple)) or len(entry)!=3:
                raise ValueError('LED entries must be [left|right,x,y] or null')
            side,x,y=entry
            if (side not in ('left','right') or isinstance(x,bool) or isinstance(y,bool)
                    or not isinstance(x,int) or not isinstance(y,int)
                    or not 0<=x<self.width or not 0<=y<self.height):
                raise ValueError('Invalid side/pixel coordinate in panel map')
            self.x[i]=x+(self.width if side=='right' else 0);self.y[i]=y;self.valid[i]=True

    @classmethod
    def load(cls,path):return cls(json.loads(Path(path).expanduser().read_text()))

    def rgb_bytes(self,data,width,height,flip_second_screen=False):
        if width!=self.width*2 or height!=self.height:
            raise ValueError('Frame dimensions do not match the two-side atlas map')
        if flip_second_screen:raise ValueError('Side-specific atlas must not request a second mirror')
        frame=np.frombuffer(memoryview(data),dtype=np.uint8) if not isinstance(data,list) else np.asarray(data,dtype=np.uint8)
        if frame.size!=width*height*3:raise ValueError('BGR payload length mismatch')
        frame=frame.reshape(height,width,3)
        pixels=frame[self.y,self.x,::-1].copy();pixels[~self.valid]=0
        return pixels.tobytes()
