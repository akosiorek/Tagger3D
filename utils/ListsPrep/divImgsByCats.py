'''
Created on 22 sie 2013

@author: adam
'''

import getopt, sys, os, exceptions, shutil, re

def main():
    
    root = ""
    pattern = r"([a-z]+)"
    
    try:
        
        opts, args = getopt.getopt(sys.argv[1:], "r:")
        
    except getopt.GetoptError:        
        print "python divImgsByCats.py -r <img root directory>"
        sys.exit(2)
        
    if len(opts) < 1:
        print "python divImgsByCats.py -r <img root directory>"
        sys.exit(2)
        
    for opt, arg in opts:
        if opt == "-r":
            root = arg
        
    imgs = os.listdir(root);
    os.chdir(root)    
    for img in imgs:
        
        #category = str(img).split("_")[0]
        match = re.match(pattern, img, re.I)
        if match:
            category = match.group(0)
        if not os.path.exists(category):
            os.mkdir(category)
        
        shutil.move(img, os.path.join(root, category))
            
if __name__ == "__main__":
    main()
            
    
    
    
    