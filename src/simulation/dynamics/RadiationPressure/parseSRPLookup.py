# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
#

from xml.etree import ElementTree

import numpy as np


class SRPLookupTableHandler:
    """Class to handle an SRP Lookup table"""
    def __init__(self):
        self.sHatBLookup = np.zeros([1, 3])
        self.forceBLookup = np.zeros([1, 3])
        self.torqueBLookup = np.zeros([1, 3])

    def parseAndLoadXML(self, filePath):
        document = ElementTree.parse(filePath)

        sHatBTree = document.find('sHatBValues')
        forceBTree = document.find('forceBValues')
        torqueBTree = document.find('torqueBValues')

        self.sHatBLookup.resize([len(list(sHatBTree)), 3])
        self.forceBLookup.resize([len(list(forceBTree)), 3])
        self.torqueBLookup.resize([len(list(torqueBTree)), 3])

        for node in list(sHatBTree):
            idx = int(node.attrib['index'])
            for value in list(node):
                if value.tag == 'value_1':
                    self.sHatBLookup[idx, 0] = value.text
                if value.tag == 'value_2':
                    self.sHatBLookup[idx, 1] = value.text
                if value.tag == 'value_3':
                    self.sHatBLookup[idx, 2] = value.text

        for node in list(forceBTree):
            idx = int(node.attrib['index'])
            for value in list(node):
                if value.tag == 'value_1':
                    self.forceBLookup[idx, 0] = value.text
                if value.tag == 'value_2':
                    self.forceBLookup[idx, 1] = value.text
                if value.tag == 'value_3':
                    self.forceBLookup[idx, 2] = value.text

        for node in list(torqueBTree):
            idx = int(node.attrib['index'])
            for value in list(node):
                if value.tag == 'value_1':
                    self.torqueBLookup[idx, 0] = value.text
                if value.tag == 'value_2':
                    self.torqueBLookup[idx, 1] = value.text
                if value.tag == 'value_3':
                    self.torqueBLookup[idx, 2] = value.text
