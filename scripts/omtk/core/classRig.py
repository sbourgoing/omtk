from maya import cmds
import pymel.core as pymel
import time

from omtk.modules import rigFaceAvarGrps
from omtk.core.classCtrl import BaseCtrl
from omtk.core.classNode import Node
from omtk.core import className
from omtk.core import classModule
from omtk.libs import libPymel
from omtk.libs import libPython
from omtk.libs import libRigging
import logging
log = logging.getLogger('omtk')

class CtrlRoot(BaseCtrl):
    """
    The main ctrl. Support global uniform scaling only.
    """
    def __init__(self, *args, **kwargs):
        super(CtrlRoot, self).__init__(create_offset=False, *args, **kwargs)

    def __createNode__(self, size=10, *args, **kwargs):
        """
        Create a wide circle.
        """
        # use meshes boundinx box
        node = pymel.circle(*args, **kwargs)[0]
        make = node.getShape().create.inputs()[0]
        make.radius.set(size)
        make.normal.set((0,1,0))

        return node

    def build(self, *args, **kwargs):
        super(CtrlRoot, self).build(*args, **kwargs)

        # Add a globalScale attribute to replace the sx, sy and sz.
        if not self.node.hasAttr('globalScale'):
            pymel.addAttr(self.node, longName='globalScale', k=True, defaultValue=1.0, minValue=0.001)
            pymel.connectAttr(self.node.globalScale, self.node.sx)
            pymel.connectAttr(self.node.globalScale, self.node.sy)
            pymel.connectAttr(self.node.globalScale, self.node.sz)
            self.node.s.set(lock=True, channelBox=False)


    @staticmethod
    def _get_recommended_radius(min_size=1.0):
        """
        Analyze the scene and return the recommended radius using the scene geometry.
        """
        geometries = pymel.ls(type='mesh')

        if not geometries:
            log.warning("Can't find any geometry in the scene.")
            return min_size

        geometries_mel = [geo.__melobject__() for geo in geometries]

        x_min, y_min, z_min, x_max, y_max, z_max = cmds.exactWorldBoundingBox(*geometries_mel)

        return max(
            min_size,
            x_max - x_min,
            z_max - z_min
        ) / 2.0

class Rig(object):
    DEFAULT_NAME = 'untitled'

    #
    # className.BaseNomenclature implementation
    #

    def _get_nomenclature_cls(self):
        """
        :return: Return the nomenclature type class that will determine the production specific nomenclature to use.
        """
        return className.BaseName

    @property
    def nomenclature(self, *args, **kwargs):
        """
        Singleton that will return the nomenclature to use.
        """
        return self._get_nomenclature_cls(*args, **kwargs)

    #
    # collections.MutableSequence implementation
    #
    def __getitem__(self, item):
        self.modules.__getitem__(item)

    def __setitem__(self, index, value):
        self.modules.__setitem__(index, value)

    def __delitem__(self, index):
        self.modules.__delitem__(index)

    def __len__(self):
        return self.modules.__len__()

    def insert(self, index, value):
        self.modules.insert(index, value)
        value._parent = self # Store the parent for optimized network serialization (see libs.libSerialization)

    def __iter__(self):
        return iter(self.modules)

    def __init__(self, name=None):
        self.name = name if name else self.DEFAULT_NAME
        self.modules = []
        self.grp_anm = None
        self.grp_geo = None
        self.grp_jnt = None
        self.grp_rig = None
        self.layer_anm = None
        self.layer_geo = None
        self.layer_rig = None

    def __str__(self):
        return '{0} <{1}>'.format(self.name, self.__class__.__name__)

    #
    # libSerialization implementation
    #
    def __callbackNetworkPostBuild__(self):
        """
        Cleaning routine automatically called by libSerialization after a network import.
        """

        # Ensure there's no None value in the .children array.
        try:
            self.modules = filter(None, self.modules)
        except (AttributeError, TypeError):
            pass


    #
    # Main implementation
    #

    def add_module(self, cls_name, *args, **kwargs):
        #if not isinstance(part, Module):
        #    raise IOError("[Rig:AddPart] Unexpected type. Got '{0}'. {1}".format(type(part), part))

        # Resolve class to use.
        cls = libPython.get_class_def(cls_name, base_class=classModule.Module, relative=True)
        if cls is None:
            raise Exception("Cannot resolve class name '{0}'".format(cls_name))

        instance = cls(*args, **kwargs)
        self.modules.append(instance)
        return instance

    def is_built(self):
        """
        :return: True if any module dag nodes exist in the scene.
        """
        return self.grp_anm is not None and self.grp_anm.exists()

    def _clean_invalid_pynodes(self):
        fnCanDelete = lambda x: (isinstance(x, (pymel.PyNode, pymel.Attribute)) and not libPymel.is_valid_PyNode(x))
        for key, val in self.__dict__.iteritems():
            if fnCanDelete(val):
                setattr(self, key, None)
            elif isinstance(val, (list, set, tuple)):
                for i in reversed(range(len(val))):
                    if fnCanDelete(val[i]):
                        val.pop(i)
                if len(val) == 0:
                    setattr(self, key, None)

    def validate(self):
        """
        Check any module can be built with it's current configuration.
        In case of error, an exception will be raised with the necessary informations.
        """
        for module in self.modules:
            module.validate()
        return True

    def build(self, **kwargs):
        # Aboard if already built
        if self.is_built():
            log.warning("Can't build {0} because it's already built!".format(self))
            return False

        sTime = time.time()

        #
        # Prebuild
        #

        # Ensure we got a root joint
        # If needed, parent orphan joints to this one
        if not libPymel.is_valid_PyNode(self.grp_jnt):
            self.grp_jnt = next(iter(libPymel.ls_root(type='joint')), None)
            '''
            if cmds.objExists(self.nomenclature.root_jnt_name):
                self.grp_jnt = pymel.PyNode(self.nomenclature.root_jnt_name)
            else:
                self.grp_jnt = pymel.createNode('joint', name=self.nomenclature.root_jnt_name)
            '''
        #all_root_jnts.setParent(self.grp_jnt)

        # Ensure all joinst have segmentScaleComprensate deactivated.
        # This allow us to scale adequately and support video game rigs.
        # If for any mean stretch and squash are necessary, implement them on a new joint chains parented to the skeletton.
        all_jnts = libPymel.ls(type='joint')
        for jnt in all_jnts:
            jnt.segmentScaleCompensate.set(False)

        # Create main grp_anm
        if not isinstance(self.grp_anm, CtrlRoot):
            self.grp_anm = CtrlRoot()
        if not self.grp_anm.is_built():
            grp_anm_size = CtrlRoot._get_recommended_radius()
            self.grp_anm.build(size=grp_anm_size)
        self.grp_anm.rename(self.nomenclature.root_anm_name)

        # Create main grp_rig
        if not isinstance(self.grp_rig, Node):
            self.grp_rig = Node()
        if not self.grp_rig.is_built():
            self.grp_rig.build()
        self.grp_rig.rename(self.nomenclature.root_rig_name)

        #
        # Build
        #

        #try:

        modules = sorted(self.modules, key=(lambda module: libPymel.get_num_parents(module.chain_jnt.start)))
        for module in modules:
            #try:
            if not module.is_built():
                print("Building {0}...".format(module))
                module.build(self, **kwargs)
            #except Exception, e:
            #    logging.error("\n\nAUTORIG BUILD FAIL! (see log)\n")
            #    traceback.print_stack()
            #    logging.error(str(e))
            #    raise e

        # Raise warnings if a module leave junk in the scene.
        for module in self.modules:
            if module.grp_anm and not module.grp_anm.getChildren():
                cmds.warning("Found empty group {0}, please cleanup module {1}.".format(
                    module.grp_anm.longName(), module
                ))
                pymel.delete(module.grp_anm)
            if module.grp_rig and not module.grp_rig.getChildren():
                cmds.warning("Found empty group {0}, please cleanup module {1}.".format(
                    module.grp_rig.longName(), module
                ))
                pymel.delete(module.grp_rig)

        #
        # Post-build
        #

        # Prevent animators from accidentaly moving offset nodes
        for ctrl in self.iter_ctrls():
            if ctrl.offset:
                ctrl.offset.t.lock()
                ctrl.offset.r.lock()
                ctrl.offset.s.lock()

        # Parent modules grp_anm to main grp_anm
        anm_grps = [module.grp_anm for module in self.modules if module.grp_anm is not None]
        for anm_grp in anm_grps:
            anm_grp.setParent(self.grp_anm)

        # Connect globalScale attribute to each modules globalScale.
        for module in self.modules:
            if module.globalScale:
                pymel.connectAttr(self.grp_anm.globalScale, module.globalScale, force=True)

        # Constraint grp_jnts to grp_anms
        pymel.delete([module for module in self.grp_jnt.getChildren() if isinstance(module, pymel.nodetypes.Constraint)])
        pymel.parentConstraint(self.grp_anm, self.grp_jnt, maintainOffset=True)
        pymel.connectAttr(self.grp_anm.globalScale, self.grp_jnt.scaleX, force=True)
        pymel.connectAttr(self.grp_anm.globalScale, self.grp_jnt.scaleY, force=True)
        pymel.connectAttr(self.grp_anm.globalScale, self.grp_jnt.scaleZ, force=True)

        # Constraint modules grp_rig to main grp_rig
        rig_grps = [module.grp_rig for module in self.modules if module.grp_rig is not None]
        for rig_grp in rig_grps:
            rig_grp.setParent(self.grp_rig)

        # Create geo root
        all_geos = libPymel.ls_root_geos()
        if not isinstance(self.grp_geo, Node):
            self.grp_geo = Node()
        if not self.grp_geo.is_built():
            self.grp_geo.build()
        self.grp_geo.rename(self.nomenclature.root_geo_name)
        all_geos.setParent(self.grp_geo)

        # Setup displayLayers
        self.layer_anm = pymel.createDisplayLayer(name=self.nomenclature.layer_anm_name, number=1, empty=True)
        pymel.editDisplayLayerMembers(self.layer_anm, self.grp_anm, noRecurse=True)
        self.layer_anm.color.set(17)  # Yellow

        self.layer_rig = pymel.createDisplayLayer(name=self.nomenclature.layer_rig_name, number=1, empty=True)
        pymel.editDisplayLayerMembers(self.layer_rig, self.grp_rig, noRecurse=True)
        pymel.editDisplayLayerMembers(self.layer_rig, self.grp_jnt, noRecurse=True)
        self.layer_rig.color.set(13)  # Red
        #self.layer_rig.visibility.set(0)  # Hidden
        self.layer_rig.displayType.set(2)  # Frozen

        self.layer_geo = pymel.createDisplayLayer(name=self.nomenclature.layer_geo_name, number=1, empty=True)
        pymel.editDisplayLayerMembers(self.layer_geo, self.grp_geo, noRecurse=True)
        self.layer_geo.color.set(12)  # Green?
        self.layer_geo.displayType.set(2)  # Frozen

        print ("[classRigRoot.Build] took {0} ms".format(time.time() - sTime))

        return True

    def unbuild(self, **kwargs):
        """
        :param kwargs: Potential parameters to pass recursively to the unbuild method of each module.
        :return: True if successful.
        """
        # Unbuild all children
        for child in self.modules:
            if child.is_built():
                child.unbuild(**kwargs)

        # Delete anm_grp
        if isinstance(self.grp_anm, CtrlRoot) and self.grp_anm.is_built():
            self.grp_anm.unbuild()

        # Delete the rig group if it isnt used anymore
        if libPymel.is_valid_PyNode(self.grp_rig) and len(self.grp_rig.getChildren()) == 0:
            pymel.delete(self.grp_rig)
            self.grp_rig = None

        # Delete the displayLayers
        if libPymel.is_valid_PyNode(self.layer_anm):
            pymel.delete(self.layer_anm)
            self.layer_anm = None
        if libPymel.is_valid_PyNode(self.layer_geo):
            pymel.delete(self.layer_geo)
            self.layer_geo = None
        if libPymel.is_valid_PyNode(self.layer_rig):
            pymel.delete(self.layer_rig)

        # Remove any references to missing pynodes
        #HACK --> Remove clean invalid PyNode
        self._clean_invalid_pynodes()
        if self.modules is None:
            self.modules = []

        return True

    #
    # Utility methods
    #

    def get_module_by_input(self, obj):
        for module in self.modules:
            if obj in module.input:
                return module

    #
    # Facial and avars utility methods
    #

    @libPython.memoized
    def get_head_jnt(self, key=None):
        """
        Not the prettiest but used to find the head for facial rigging.
        """
        if key is None:
            def key(obj):
                name_safe = obj.stripNamespace().lower()
                return 'head' in name_safe or 'face'in name_safe

        # TODO: Find a better way!
        for jnt in pymel.ls(type='joint'):
            if key(jnt):
                return jnt

        raise Exception("Can't resolve head joint!")

    @libPython.memoized
    def get_jaw_jnt(self, key=None):
        """
        Not the prettiest but used to find the jaw for facial rigging.
        """
        if key is None:
            def key(obj):
                name_safe = obj.stripNamespace().lower()
                return 'jaw' in name_safe

        # TODO: Find a better way!
        for jnt in pymel.ls(type='joint'):
            if key(jnt):
                return jnt

        raise Exception("Can't resolve jaw joint!")

    @libPython.memoized
    def get_face_macro_ctrls_distance_from_head(self, multiplier=1.2):
        """
        :return: The recommended distance between the head middle and the face macro ctrls.
        """
        return 20
        '''
        jnt_head = self.get_head_jnt()
        ref_tm = jnt_head.getMatrix(worldSpace=True)

        geometries = libRigging.get_affected_geometries(jnt_head)

        # Resolve the top of the head location
        pos = pymel.datatypes.Point(ref_tm.translate)
        #dir = pymel.datatypes.Point(1,0,0) * ref_tm
        #dir = dir.normal()
        # This is strange but not pointing to the world sometime don't work...
        # TODO: FIX ME
        dir = pymel.datatypes.Point(0,1,0)

        top = next(iter(libRigging.ray_cast(pos, dir, geometries)), None)
        if not top:
            raise Exception("Can't resolve head top location using raycasts!")

        # Resolve the middle of the head
        middle = ((top-pos) * 0.5) + pos

        # Find the front of the face
        # For now, one raycase seem fine.
        #dir = pymel.datatypes.Point(0,-1,0) * ref_tm
        #dir.normalize()
        dir = pymel.datatypes.Point(0,0,1)
        front = next(iter(libRigging.ray_cast(middle, dir, geometries)), None)
        if not front:
            raise Exception("Can't resolve head front location using raycasts!")

        distance = libPymel.distance_between_vectors(middle, front)

        return distance * multiplier
        '''

    @libPython.memoized
    def get_head_length(self):
        jnt_head = self.get_head_jnt()
        ref_tm = jnt_head.getMatrix(worldSpace=True)

        geometries = libRigging.get_affected_geometries(jnt_head)

        # Resolve the top of the head location
        bot = pymel.datatypes.Point(ref_tm.translate)
        #dir = pymel.datatypes.Point(1,0,0) * ref_tm
        #dir = dir.normal()
        # This is strange but not pointing to the world sometime don't work...
        # TODO: FIX ME
        dir = pymel.datatypes.Point(0,1,0)

        top = next(iter(libRigging.ray_cast(bot, dir, geometries)), None)
        if not top:
            raise Exception("Can't resolve head top location using raycasts!")

        return libPymel.distance_between_vectors(bot, top)

    def iter_avars(self):
        """
        Iterate through all avars module of the rig.
        """
        from omtk.modules import rigFaceAvar
        for module in self.modules:
            if isinstance(module, classAvar.AvarFollicle):
                yield module
            elif isinstance(module, rigFaceAvarGrps.ModuleFace):
                for avar in module.avars:
                    yield avar

    def iter_ctrls(self):
        for module in self.modules:
            for ctrl in module.get_ctrls(recursive=True):
                yield ctrl

