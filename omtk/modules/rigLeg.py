import pymel.core as pymel
from maya import cmds

from omtk.modules.rigArm import Arm
from omtk.modules.rigIK import IK
from omtk.modules import rigIK
from omtk.libs import libRigging
from omtk.libs import libCtrlShapes

class CtrlIkLeg(rigIK.CtrlIk):
    def __createNode__(self, *args, **kwargs):
        return libCtrlShapes.create_shape_box_feet(*args, **kwargs)


class FootRollIK(IK):
    #_CLASS_CTRL_IK = CtrlIkLeg

    """
    A standard footroll that remember it's pivot when building/unbuilding.
    """

    def __init__(self, *args, **kwargs):
        super(FootRollIK, self).__init__(*args, **kwargs)

        self.pivot_ankle = None
        self.pivot_front = None
        self.pivot_back = None
        self.pivot_inn = None
        self.pivot_out = None

        self.pivot_ankle_pos = None
        self.pivot_front_pos = None
        self.pivot_back_pos = None
        self.pivot_inn_pos = None
        self.pivot_out_pos = None

    def _get_reference_plane(self):
        """
        When holding/fetching the footroll pivots, we do not want to use their worldSpace transforms.
        :return: The reference worldSpace matrix to use when holding/fetching pivot positions.
        """
        pos = pymel.datatypes.Point(self.chain_jnt[self.iCtrlIndex].getTranslation(space='world'))
        tm = pymel.datatypes.Matrix(
            1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            pos.x, pos.y, pos.z, 1
        )
        return tm

        #return self.chain_jnt[0].getMatrix(worldSpace=True)

    def _get_recommended_pivot_front(self, tm_ref, pos_toes, pos_tip):
        """
        Determine recommended position using ray-cast from the toes.
        If the ray-cast fail, use the last joint position.
        """
        geometries = pymel.ls(type='mesh')
        results = libRigging.ray_cast(pos_toes, pymel.datatypes.Vector(0, 0, 1), geometries)
        pos = next(iter(results), None)
        if not pos:
            cmds.warning("Can't automatically solve FootRoll front pivot, using last joint as reference.")
            pos = pos_tip
        else:
            # Compare our result with the last joint position and take the longuest.
            pos.z = max(pos.z, pos_tip.z)

        # Ensure we are aligned with the reference matrix.
        pos_relative = pos * tm_ref.inverse()
        pos_relative.x = 0
        pos_relative.y = 0
        pos = pos_relative * tm_ref

        return pos

    def _get_recommended_pivot_back(self, tm_ref, pos_toes):
        """
        Determine recommended position using ray-cast from the toes.
        If the ray-cast fail, use the toes position.
        """
        geometries = pymel.ls(type='mesh')  # TODO: Use a property in the Rig class?
        results = libRigging.ray_cast(pos_toes, pymel.datatypes.Vector(0, 0, -1), geometries)
        pos = next(iter(results), None)
        if not pos:
            cmds.warning("Can't automatically solve FootRoll back pivot.")
            pos = pos_toes

        # Ensure we are aligned with the reference matrix.
        pos_relative = pos * tm_ref.inverse()
        pos_relative.x = 0
        pos_relative.y = 0
        pos = pos_relative * tm_ref

        return pos

    def _get_recommended_pivot_bank(self, tm_ref, pos_toes, direction=1):
        """
        Determine recommended position using ray-cast from the toes.
        TODO: If the ray-case fail, use a specified default value.
        """
        geometries = pymel.ls(type='mesh')  # TODO: Use a property in the Rig class?
        results = libRigging.ray_cast(pos_toes, pymel.datatypes.Vector(direction, 0, 0), geometries)
        pos = next(iter(results), None)
        if not pos:
            cmds.warning("Can't automatically solve FootRoll bank inn pivot.")
            pos = pos_toes

        return pos

    def build(self, rig, attr_holder=None, **kwargs):
        if len(self.chain_jnt) != 5:
            raise Exception("Unexpected input count for {0}. Expected 5, got {1}.".format(
                self, len(self.chain_jnt)
            ))

        super(FootRollIK, self).build(rig, orient_ik_ctrl=False, **kwargs)

        nomenclature_rig = self.get_nomenclature_rig(rig)

        jnt_foot, jnt_toes, jnt_tip = self._chain_ik[self.iCtrlIndex:]

        # Create FootRoll (chain?)
        pos_foot = pymel.datatypes.Point(jnt_foot.getTranslation(space='world'))
        pos_toes = pymel.datatypes.Point(jnt_toes.getTranslation(space='world'))
        pos_tip = pymel.datatypes.Point(jnt_tip.getTranslation(space='world'))


        # Resolve pivot locations
        tm_ref = self._get_reference_plane()

        # Pivot Ankle
        self.pivot_ankle = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotAnkle'))
        if self.pivot_ankle_pos:
            self.pivot_ankle.setTranslation(pymel.datatypes.Point(self.pivot_ankle_pos) * tm_ref, space='world')
        else:
            self.pivot_ankle.t.set(pos_toes)

        # Pivot Front
        self.pivot_front = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotFront'))
        if self.pivot_front_pos:
            self.pivot_front.setTranslation(pymel.datatypes.Point(self.pivot_front_pos) * tm_ref, space='world')
        else:
            pos_pivot_front = self._get_recommended_pivot_front(tm_ref, pos_toes, pos_tip)
            self.pivot_front.t.set(pos_pivot_front)
            self.pivot_front.ty.set(0)

        # Pivot Back
        self.pivot_back = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotBack'))
        if self.pivot_back_pos:
            self.pivot_back.setTranslation(pymel.datatypes.Point(self.pivot_back_pos) * tm_ref, space='world')
        else:
            pos_pivot_back = self._get_recommended_pivot_back(tm_ref, pos_toes)
            self.pivot_back.setTranslation(pos_pivot_back)
            self.pivot_back.ty.set(0)

        # Pivot Bank Inn
        self.pivot_inn = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotInn'))
        if self.pivot_inn_pos:
            self.pivot_inn.setTranslation(pymel.datatypes.Point(self.pivot_inn_pos) * tm_ref, space='world')
        else:
            pos_pivot_inn = self._get_recommended_pivot_bank(tm_ref, pos_toes, direction=-1)
            self.pivot_inn.t.set(pos_pivot_inn)
            self.pivot_inn.ty.set(0)

        # Pivot Bank Out
        self.pivot_out = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotOut'))
        if self.pivot_out_pos:
            self.pivot_out.setTranslation(pymel.datatypes.Point(self.pivot_out_pos) * tm_ref, space='world')
        else:
            pos_pivot_out = self._get_recommended_pivot_bank(tm_ref, pos_toes, direction=1)
            self.pivot_out.t.set(pos_pivot_out)
            self.pivot_out.ty.set(0)

        root_footRoll = pymel.createNode('transform', name=nomenclature_rig.resolve('footRoll'))
        chain_footroll = [root_footRoll, self.pivot_inn, self.pivot_out, self.pivot_back, self.pivot_front,
                          self.pivot_ankle]
        libRigging.create_hyerarchy(chain_footroll)
        chain_footroll[0].setParent(self.grp_rig)

        # Create attributes
        attr_holder = self.ctrl_ik
        pymel.addAttr(attr_holder, longName='rollAuto', k=True)
        pymel.addAttr(attr_holder, longName='rollAutoThreshold', k=True, defaultValue=45)
        pymel.addAttr(attr_holder, longName='bank', k=True)
        pymel.addAttr(attr_holder, longName='rollAnkle', k=True)
        pymel.addAttr(attr_holder, longName='rollFront', k=True)
        pymel.addAttr(attr_holder, longName='rollBack', k=True)

        attr_inn_roll_auto = attr_holder.attr('rollAuto')
        attr_inn_roll_auto_threshold = attr_holder.attr('rollAutoThreshold')
        attr_inn_bank = attr_holder.attr('bank')
        attr_inn_roll_ankle = attr_holder.attr('rollAnkle')
        attr_inn_roll_front = attr_holder.attr('rollFront')
        attr_inn_roll_back = attr_holder.attr('rollBack')

        attr_roll_auto_m = libRigging.create_utility_node('condition', operation=2, firstTerm=attr_inn_roll_auto,
                                                          secondTerm=attr_inn_roll_auto_threshold,
                                                          colorIfTrueR=attr_inn_roll_auto_threshold,
                                                          colorIfFalseR=attr_inn_roll_auto
                                                          ).outColorR  # Less
        attr_roll_auto_f = libRigging.create_utility_node('condition', operation=2,
                                                          firstTerm=attr_inn_roll_auto,
                                                          secondTerm=attr_inn_roll_auto_threshold,
                                                          colorIfFalseR=0,
                                                          colorIfTrueR=(
                                                              libRigging.create_utility_node('plusMinusAverage',
                                                                                             operation=2,
                                                                                             input1D=[
                                                                                                 attr_inn_roll_auto,
                                                                                                 attr_inn_roll_auto_threshold]).output1D)
                                                          ).outColorR  # Substract
        attr_roll_auto_b = libRigging.create_utility_node('condition', operation=2, firstTerm=attr_inn_roll_auto,
                                                          secondTerm=0.0,
                                                          colorIfTrueR=0, colorIfFalseR=attr_inn_roll_auto
                                                          ).outColorR  # Greater

        attr_roll_m = libRigging.create_utility_node('addDoubleLinear', input1=attr_roll_auto_m,
                                                     input2=attr_inn_roll_ankle).output
        attr_roll_f = libRigging.create_utility_node('addDoubleLinear', input1=attr_roll_auto_f,
                                                     input2=attr_inn_roll_front).output
        attr_roll_b = libRigging.create_utility_node('addDoubleLinear', input1=attr_roll_auto_b,
                                                     input2=attr_inn_roll_back).output

        attr_bank_inn = libRigging.create_utility_node('condition', operation=2,
                                                       firstTerm=attr_inn_bank, secondTerm=0,
                                                       colorIfTrueR=attr_inn_bank,
                                                       colorIfFalseR=0.0
                                                       ).outColorR  # Greater

        attr_bank_out = libRigging.create_utility_node('condition', operation=4,
                                                       firstTerm=attr_inn_bank, secondTerm=0,
                                                       colorIfTrueR=attr_inn_bank,
                                                       colorIfFalseR=0.0).outColorR  # Less

        pymel.connectAttr(attr_roll_m, self.pivot_ankle.rotateX)
        pymel.connectAttr(attr_roll_f, self.pivot_front.rotateX)
        pymel.connectAttr(attr_roll_b, self.pivot_back.rotateX)
        pymel.connectAttr(attr_bank_inn, self.pivot_inn.rotateZ)
        pymel.connectAttr(attr_bank_out, self.pivot_out.rotateZ)

        # Create ikHandles
        ikHandle_foot, ikEffector_foot = pymel.ikHandle(startJoint=jnt_foot, endEffector=jnt_toes, solver='ikSCsolver')
        ikHandle_foot.rename(nomenclature_rig.resolve('ikHandle', 'foot'))
        ikHandle_foot.setParent(self.grp_rig)
        ikHandle_toes, ikEffector_toes = pymel.ikHandle(startJoint=jnt_toes, endEffector=jnt_tip, solver='ikSCsolver')
        ikHandle_toes.rename(nomenclature_rig.resolve('ikHandle', 'ties'))
        ikHandle_toes.setParent(self.grp_rig)

        # Parent ikHandlers
        # Note that we are directly parenting them so the 'Preserve Child Transform' of the translate tool still work.
        ikHandle_foot.setParent(self.pivot_front)
        ikHandle_toes.setParent(self.pivot_back)

        # Hack: Re-constraint foot ikhandle
        # todo: cleaner!
        pymel.parentConstraint(self.ctrl_ik, root_footRoll, maintainOffset=True)

        # Connect ikHandles to footroll
        fn_can_delete = lambda x: isinstance(x, pymel.nodetypes.Constraint) and \
                                  not isinstance(x, pymel.nodetypes.PoleVectorConstraint)
        pymel.delete(filter(fn_can_delete, self._ik_handle.getChildren()))
        pymel.parentConstraint(self.pivot_ankle, self._ik_handle, maintainOffset=True)

        '''
        # Constraint swivel to ctrl_ik
        pymel.parentConstraint(self.ctrl_ik, self.ctrl_swivel,
                               maintainOffset=True)  # TODO: Implement SpaceSwitch
        '''

        # Handle globalScale
        pymel.connectAttr(self.grp_rig.globalScale, root_footRoll.scaleX)
        pymel.connectAttr(self.grp_rig.globalScale, root_footRoll.scaleY)
        pymel.connectAttr(self.grp_rig.globalScale, root_footRoll.scaleZ)

    def unbuild(self):
        # Remember footroll locations in relation with a safe matrix
        # The reference matrix is the ankle, maybe we should zero out the y axis.
        tm_ref_inv = self._get_reference_plane().inverse()

        self.pivot_ankle_pos = (self.pivot_ankle.getMatrix(worldSpace=True) * tm_ref_inv).translate
        self.pivot_front_pos = (self.pivot_front.getMatrix(worldSpace=True) * tm_ref_inv).translate
        self.pivot_back_pos = (self.pivot_back.getMatrix(worldSpace=True) * tm_ref_inv).translate
        self.pivot_inn_pos = (self.pivot_inn.getMatrix(worldSpace=True) * tm_ref_inv).translate
        self.pivot_out_pos = (self.pivot_out.getMatrix(worldSpace=True) * tm_ref_inv).translate

        super(FootRollIK, self).unbuild()

        self.pivot_ankle = None
        self.pivot_front = None
        self.pivot_back = None
        self.pivot_inn = None
        self.pivot_out = None


class Leg(Arm):
    def __init__(self, *args, **kwargs):
        super(Leg, self).__init__(*args, **kwargs)
        self.sysFootRoll = None

    def _create_sys_ik(self, rig, **kwargs):
        if not isinstance(self.sysIK, FootRollIK):
            self.sysIK = FootRollIK(self.chain_jnt)
        self.sysIK.build(rig, **kwargs)
