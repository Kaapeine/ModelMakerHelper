#!/usr/bin/env python

import os
# from bs4 import BeautifulSoup, Comment

if __name__ == "__main__":
    home = os.path.expanduser("~")
    f = open(os.path.join(home, "temp.urdf"), "r")
    urdf = BeautifulSoup(f, 'lxml')

    # To get the path of the real URDF 
    # comment = soup.find_all(text=lambda text:isinstance(text, Comment))[1]
    

    
    

