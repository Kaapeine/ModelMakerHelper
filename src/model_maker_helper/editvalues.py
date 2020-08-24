#!/usr/bin/env python

import rospy
import os
import yaml

def openyamlfile(yamlpath):
    with open(yamlpath[0]) as f:
        return yaml.load(f)
      
def editvalues(labeldict, valdict, yamlpath):
    yamlfile = openyamlfile(yamlpath)
    count = len(labeldict)
    yamlfilekeys = yamlfile.keys()
    
    for c in range(count):
        if not valdict[c]:
            return False
        
        if labeldict[c] in yamlfilekeys:
            yamlfile.update({labeldict[c] : int(valdict[c])})
        
    with open(yamlpath[0], 'w') as f:
        yaml.dump(yamlfile, f)
        
    return True
        
    
        
        
    


    

    
    

