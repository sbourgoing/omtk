import functools
import logging

from omtk.libs import libSerialization

import pymel.core as pymel

def CallFnOnNetworkByClass(_sFn, _sCls):
    fnFilter = lambda x: libSerialization.isNetworkInstanceOfClass(x, _sCls)
    networks = libSerialization.getConnectedNetworks(pymel.selected(), key=fnFilter)
    for network in networks:
        rigPart = libSerialization.importFromNetwork(network)

        if not hasattr(rigPart, _sFn):
            logging.warning("Can't find attribute {0} in {1}".format(_sFn, network)); continue

        try:
            getattr(rigPart, _sFn)()
        except Exception, e:
            print str(e)

switchToIk = functools.partial(CallFnOnNetworkByClass, 'switchToIk', 'Arm')
switchToFk = functools.partial(CallFnOnNetworkByClass, 'switchToFk', 'Arm')