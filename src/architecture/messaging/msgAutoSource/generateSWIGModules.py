#!/usr/bin/env python3
import sys

if __name__ == "__main__":
     moduleOutputPath = sys.argv[1]
     swigTemplateFile = sys.argv[2]
     structType = sys.argv[3].split('Payload')[0]
     header = sys.argv[4]

     swigFid = open(swigTemplateFile, 'r')
     swigTemplateData = swigFid.read()
     swigFid.close()

     moduleFileOut = open(moduleOutputPath, 'w')
     moduleFileOut.write(swigTemplateData.format(type=structType, header=header))
     moduleFileOut.close()
