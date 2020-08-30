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
        
        print(labeldict[c])
        
        if str(labeldict[c]) in str(yamlfilekeys):
            pos = yamlfilekeys.index(labeldict[c]) # position of labeldict[c] in yamlfile
            yamlfile.update({str(yamlfilekeys[pos]) : float(valdict[c])})
        
    with open(yamlpath[0], 'w') as f:
        yaml.dump(yamlfile, f)
        
    return True
        
    
        
        
    


    

    
    

