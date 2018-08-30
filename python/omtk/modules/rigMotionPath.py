import pymel.core as pymel
from omtk.core.classModule import Module
from omtk.core.classCtrl import BaseCtrl
from omtk.libs import libPython
from omtk.libs import libRigging
from omtk.libs import libAttr
from omtk.libs import libPymel
from omtk.libs import libCtrlShapes


class CtrlMotionPath(BaseCtrl):
    """
    Ctrl definition use for motion path setup
    """
    def __createNode__(self, *args, **kwargs):
        node = super(CtrlMotionPath, self).__createNode__(multiplier=1.1, *args, **kwargs)

        make = next(iter(node.inputs()), None)
        if make:
            # TODO: Multiply radius???
            # make.radius.set(size)
            make.degree.set(1)
            make.sections.set(8)

        return node


class CtrlRootMotionPath(BaseCtrl):
    def __createNode__(self, *args, **kwargs):
        size = self._get_recommended_size(**kwargs)
        kwargs['size'] = size
        node = libCtrlShapes.create_square(**kwargs)
        # node.drawOverride.overrideEnabled.set(1)
        # node.drawOverride.overrideColor.set(25)
        return node


class MotionPath(Module):
    """
    Module that will use input joint to create a curve and setup a motion path on it
    """

    PATH_ATTR_NAME = 'path'

    def __init__(self, *args, **kwargs):
        super(MotionPath, self).__init__(*args, **kwargs)
        self.ctrls = []
        self.master_ctrl = None

    def build(self, *args, **kwargs):
        """
        Build fk ctrl over all the joints found in the module chain

        TODO - Fix problem when rotation root
        TODO - Setup managment of roll

        :param args: Additional args
        :param kwargs: Additional keywords args
        """

        super(MotionPath, self).build(*args, **kwargs)

        nomenclature_anm = self.get_nomenclature_anm()
        nomenclature_rig = self.get_nomenclature_rig()

        nomenclature_rig_grp = self.get_nomenclature_rig_grp()

        # Initialize ctrls
        libPython.resize_list(self.ctrls, len(self.jnts))
        for i, ctrl in enumerate(self.ctrls):
            self.ctrls[i] = self.init_ctrl(CtrlMotionPath, ctrl)

        self.master_ctrl = self.init_ctrl(CtrlRootMotionPath, self.master_ctrl)

        # If chained, we already have our order
        if len(self.chains) == 1:
            ordered_jnts = self.chains[0][:]
        else:
            # Since we have multiple chains (probably loose joints), we will try to order them by name
            # TODO - Find a better way to manage that ?
            ordered_jnts = sorted(self.jnts, key=lambda x: str(x.stripNamespace()))

        # Setup the master ctrl, since it will drive the curve
        master_ctrl_name = nomenclature_anm.resolve('master')
        self.master_ctrl.build(name=master_ctrl_name, refs=ordered_jnts[0], geometries=self.rig.get_meshes())
        self.master_ctrl.setMatrix(ordered_jnts[0].getMatrix(worldSpace=True))
        self.master_ctrl.setParent(self.grp_anm)

        # Create a curve from nodes with a degree of 1, turn it to bezier and smooth it
        curve = next(
            (input for input in self.input if libPymel.isinstance_of_shape(input, pymel.nodetypes.NurbsCurve)), None)
        if not curve:
            curve = libRigging.create_curve_from_nodes(ordered_jnts, 1)
            pymel.nurbsCurveToBezier()
            pymel.smoothCurve('{0}.cv[*]'.format(curve.getShape().name()), rpo=True, s=10)
            pymel.rename(curve, nomenclature_rig.resolve("curve"))

        curve_grp_name = nomenclature_rig_grp.resolve("curve")
        curve_grp = pymel.createNode('transform', name=curve_grp_name, parent=self.grp_rig)
        curve.setParent(curve_grp)

        pymel.parentConstraint(self.master_ctrl, curve_grp, mo=True)

        # Create a path attribute to drive the motion path U value
        if not pymel.hasAttr(self.master_ctrl.node, self.PATH_ATTR_NAME):
            path_value_attr = libAttr.addAttr(
                self.master_ctrl,
                defaultValue=100.0,
                longName=self.PATH_ATTR_NAME,
                k=True,
                maxValue=100.0,
                minValue=0.0
            )
        else:
            path_value_attr = self.master_ctrl.path

        util_mult_path_percent = libRigging.create_utility_node(
            'multiplyDivide',
            operation=2,
            input2X=100.0
        )

        pymel.connectAttr(path_value_attr, util_mult_path_percent.input1X)

        # Create reference node used to be driven by the motion path
        ref_grp_name = nomenclature_rig_grp.resolve("ref_offset")
        ref_grp = pymel.createNode('transform', name=ref_grp_name, parent=self.grp_rig)

        data = []
        # Build chain ctrls
        for j, jnt in enumerate(ordered_jnts):
            ctrl = self.ctrls[j]

            # Resolve ctrl name.
            # TODO: Validate with multiple chains
            ctrl_name = nomenclature_anm.resolve('{0:02d}'.format(j))
            ctrl.build(name=ctrl_name, refs=jnt, geometries=self.rig.get_meshes())
            ctrl.setMatrix(jnt.getMatrix(worldSpace=True))
            ctrl.setParent(self.grp_anm)

            pymel.parentConstraint(ctrl, jnt, maintainOffset=True)
            # pymel.connectAttr(ctrl.scaleX, jnt.scaleX)
            # pymel.connectAttr(ctrl.scaleY, jnt.scaleY)
            # pymel.connectAttr(ctrl.scaleZ, jnt.scaleZ)

            # Also create a ref node use for the motion path
            ref_name = nomenclature_rig.resolve("ref_offset{0:02d}".format(j))
            ref = pymel.createNode('transform', name=ref_name, parent=ref_grp)

            ref.setMatrix(jnt.getMatrix(worldSpace=True), worldSpace=True)

            # Find and position the ref node the closest point on curve found
            param, pos, normal = libRigging.closest_point_on_curve(ref, curve.getShape(), debug=False)
            ref.setTranslation(pos, worldSpace=True)
            if j != 0 and j != len(self.jnts) - 1:
                ref.setRotation(normal, worldSpace=True)
            data.append((param, ref))

        # With the different param information, compute the motion path U value for each nodes
        # and setup the motion path
        for (param, ref), ctrl in zip(data, self.ctrls):
            util_mult_u_value = libRigging.create_utility_node(
                'multiplyDivide',
                input1X=param
            )

            pymel.connectAttr(util_mult_path_percent.outputX, util_mult_u_value.input2X)

            self.create_motionpath_node(curve.getShape(), ref,  u_value_attr=util_mult_u_value.outputX,
                                        follow=True, frontAxis='x', upAxis='y')

            # Finally constraint the ref on the ctrl offset
            libAttr.unlock_trs(ctrl.offset)
            pymel.parentConstraint(ref, ctrl.offset, mo=True)

    def create_motionpath_node(self, curve_shape, output_node, u_value_attr=None, constraint_rot=True, *args, **kwargs):
        """
        Manually create and connect only what is needed in the motion path node

        We remove a addDoubleLinear that is connected with the node Trans Minus Rotate Pivot. From what I have seen in
        this setup, this connection is not needed and can cause 'false cycle' since there is a warning but not visual
        problem

        :param curve_shape: The shape use for the motion path
        :param output_node: The node that will be constraint on the motion path
        :param u_value_attr: The attribute use to drive the u_value. If none, it will use the default behavior
        :param constraint_rot: Is the motion path rotate data will be used to drive the rotation of the node
        :param args: Additional arguments
        :param kwargs: Additional keyword arguments

        :return: The motion path node created
        """

        #TODO - Could support different way to deal with twists

        mp = pymel.createNode('motionPath')
        curve_shape.worldSpace[0].connect(mp.geometryPath)
        self.grp_anm.worldMatrix.connect(mp.worldUpMatrix)
        mp.follow.set(True)
        mp.frontAxis.set(0)
        mp.upAxis.set(1)
        mp.worldUpType.set(2)
        if u_value_attr:
            pymel.disconnectAttr(mp.uValue)
            pymel.connectAttr(u_value_attr, mp.uValue)

        # Rotate order and rotation
        pymel.connectAttr(mp.rotateOrder, output_node.rotateOrder)
        if constraint_rot:
            pymel.connectAttr(mp.rotateX, output_node.rotateX)
            pymel.connectAttr(mp.rotateY, output_node.rotateY)
            pymel.connectAttr(mp.rotateZ, output_node.rotateZ)

        # Translation
        pymel.connectAttr(mp.xCoordinate, output_node.translateX)
        pymel.connectAttr(mp.yCoordinate, output_node.translateY)
        pymel.connectAttr(mp.zCoordinate, output_node.translateZ)

        return mp

    def unbuild(self):
        """
        Unbuild the module
        """

        # Delete the ctrls in reverse hyerarchy order. and after unbuild the master ctrl
        ctrls = self.get_ctrls()
        ctrls = filter(libPymel.is_valid_PyNode, ctrls)
        ctrls = reversed(sorted(ctrls, key=libPymel.get_num_parents))
        for ctrl in ctrls:
            ctrl.unbuild()

        self.master_ctrl.unbuild()

        super(MotionPath, self).unbuild(disconnect_attr=True)

    def validate(self):
        """
        Allow the ui to know if the module is valid to be builded or not
        :return: True or False depending if it pass the building validation
        """
        super(MotionPath, self).validate()

        return True

def register_plugin():
    return MotionPath
