import pymel.core as pymel
from omtk.core.classModule import Module
from omtk.core.classCtrl import BaseCtrl
from omtk.libs import libPython
from omtk.libs import libRigging
from omtk.libs import libAttr
from omtk.libs import libPymel


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


class MotionPath(Module):
    """
    Module that will use input joint to create a curve and setup a motion path on it
    """
    def __init__(self, *args, **kwargs):
        super(MotionPath, self).__init__(*args, **kwargs)
        self.ctrls = []

    def build(self, *args, **kwargs):
        """
        Build fk ctrl over all the joints found in the module chain

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

        # If chained, we already have our order
        ordered_jnts = None
        if len(self.chains) == 1:
            ordered_jnts = self.chains[0][:]
        else:
            # Since we have multiple chains (probably loose joints), we will try to order them by name
            # TODO - Find a better way to manage that ?
            ordered_jnts = sorted(self.jnts, key=lambda x: str(x.stripNamespace()))

        # Create a curve from nodes with a degree of 1, turn it to bezier and smooth it
        curve = next(
            (input for input in self.input if libPymel.isinstance_of_shape(input, pymel.nodetypes.NurbsCurve)), None)
        if not curve:
            curve = libRigging.create_curve_from_nodes(ordered_jnts, 1)
            pymel.nurbsCurveToBezier()
            pymel.smoothCurve('{0}.cv[*]'.format(curve.getShape().name()), rpo=True, s=10)

        curve_grp_name = nomenclature_rig_grp.resolve("curve")
        curve_grp = pymel.createNode('transform', name=curve_grp_name, parent=self.grp_rig)
        curve.setParent(curve_grp)

        pymel.parentConstraint(self.grp_anm, curve_grp)

        # Create a path attribute to drive the motion path U value
        path_value_attr = libAttr.addAttr(
            self.grp_rig,
            defaultValue=100.0,
            longName='path',
            k=True,
            maxValue=100.0,
            minValue=0.0
        )

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
            mp = pymel.PyNode(pymel.pathAnimation(ref, c=curve))
            pymel.disconnectAttr(mp.uValue)
            # mp.uValue.set(param)

            util_mult_u_value = libRigging.create_utility_node(
                'multiplyDivide',
                input1X=param
            )

            pymel.connectAttr(util_mult_path_percent.outputX, util_mult_u_value.input2X)
            pymel.connectAttr(util_mult_u_value.outputX, mp.uValue)

            # Finally constraint the ref on the ctrl offset
            libAttr.unlock_trs(ctrl.offset)
            pymel.parentConstraint(ref, ctrl.offset, mo=True)

    def unbuild(self):
        """
        Unbuild the module
        """
        
        super(MotionPath, self).unbuild()

    def validate(self):
        """
        Allow the ui to know if the module is valid to be builded or not
        :return: True or False depending if it pass the building validation
        """
        super(MotionPath, self).validate()

        self.curve = None

        return True

def register_plugin():
    return MotionPath
