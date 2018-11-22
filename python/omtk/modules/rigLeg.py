import logging

import pymel.core as pymel
from maya import cmds
from omtk import constants
from omtk.libs import libAttr
from omtk.libs import libCtrlShapes
from omtk.libs import libRigging
from omtk.modules import rigIK
from omtk.modules import rigLimb

log = logging.getLogger('omtk')


class CtrlIkLeg(rigIK.CtrlIk):
    """
    Inherit of base CtrlIk to create a specific box shaped controller
    """

    def __createNode__(self, *args, **kwargs):
        return libCtrlShapes.create_shape_box_feet(*args, **kwargs)


class LegIk(rigIK.IK):
    """
    IK/FK setup customized for Leg rigging. Include a FootRoll.
    Create an IK chain with an embeded footroll.
    Two modes are supported:
    1) leg_upp, leg_low, leg_foot, leg_toes, leg_tip (classical setup)
    2) leg_upp, leg_low, leg_foot, leg_heel, leg_toes, leg_tip (advanced setup)
    Setup #2 is more usefull if the character have shoes.
    This allow us to ensure the foot stay fixed when the 'Ankle Side' attribute is used.
    """
    _CLASS_CTRL_IK = CtrlIkLeg
    SHOW_IN_UI = False

    BACK_ROTX_LONGNAME = 'rollBack'
    BACK_ROTX_NICENAME = 'Back Roll'
    BACK_ROTY_LONGNAME = 'backTwist'
    BACK_ROTY_NICENAME = 'Back Twist'

    HEEL_ROTY_LONGNAME = 'footTwist'
    HEEL_ROTY_NICENAME = 'Heel Twist'

    ANKLE_ROTX_LONGNAME = 'rollAnkle'
    ANKLE_ROTX_NICENAME = 'Ankle Roll'
    ANKLE_ROTZ_LONGNAME = 'heelSpin'
    ANKLE_ROTZ_NICENAME = 'Ankle Side'

    TOES_ROTY_LONGNAME = 'toesTwist'
    TOES_ROTY_NICENAME = 'Toes Twist'

    TOESFK_ROTX_LONGNAME = 'toeWiggle'
    TOESFK_ROTX_NICENAME = 'Toe Wiggle'

    FRONT_ROTX_LONGNAME = 'rollFront'
    FRONT_ROTX_NICENAME = 'Front Roll'
    FRONT_ROTY_LONGNAME = 'frontTwist'
    FRONT_ROTY_NICENAME = 'Front Twist'

    AUTOROLL_THRESHOLD_LONGNAME = 'rollAutoThreshold'
    AUTOROLL_THRESHOLD_NICENAME = 'Roll Auto Threshold'

    """
    A standard footroll that remember it's pivot when building/unbuilding.
    """

    def __init__(self, *args, **kwargs):
        super(LegIk, self).__init__(*args, **kwargs)

        # Properties that contain the pivot reference object for each points.
        # This is defined when the IK is built.
        self.pivot_foot_heel = None
        self.pivot_toes_heel = None
        self.pivot_toes_ankle = None
        self.pivot_foot_front = None
        self.pivot_foot_back = None
        self.pivot_foot_inn = None
        self.pivot_foot_out = None
        self.pivot_foot_ankle = None
        self.pivot_foot_toes_fk = None

        # Properties that contain the pivot positions relative to the foot matrix.
        # This is defined when the IK is un-built.
        self.pivot_foot_heel_pos = None
        self.pivot_toes_heel_pos = None
        self.pivot_toes_ankle_pos = None
        self.pivot_foot_front_pos = None
        self.pivot_foot_back_pos = None
        self.pivot_foot_inn_pos = None
        self.pivot_foot_out_pos = None

        # Preserve the auto-threshold between builds.
        self.attrAutoRollThreshold = None

    def _get_reference_plane(self):
        """
        When holding/fetching the footroll pivots, we do not want to use their worldSpace transforms.
        :return: The reference worldSpace matrix to use when holding/fetching pivot positions.
        """
        jnts = self.input[self.iCtrlIndex:]
        pos_s = jnts[0].getTranslation(space='world')
        pos_e = jnts[-1].getTranslation(space='world')

        # We take in account that the foot is always flat on the floor.
        axis_up = pymel.datatypes.Vector(0, 1, 0)
        axis_front = pos_e - pos_s
        axis_front.y = 0
        axis_front.normalize()
        # Since we are facing another axis, we need to compute the cross product in the inverse order
        if self.rig.DEFAULT_RIG_FACING_AXIS != constants.Axis.x:
            axis_side = axis_up.cross(axis_front)
        else:
            axis_side = axis_front.cross(axis_up)
        axis_side.normalize()

        pos = pymel.datatypes.Point(self.chain_jnt[self.iCtrlIndex].getTranslation(space='world'))
        if self.rig.DEFAULT_RIG_FACING_AXIS != constants.Axis.x:
            tm = pymel.datatypes.Matrix(
                axis_side.x, axis_side.y, axis_side.z, 0,
                axis_up.x, axis_up.y, axis_up.z, 0,
                axis_front.x, axis_front.y, axis_front.z, 0,
                pos.x, pos.y, pos.z, 1
            )
        else:
            tm = pymel.datatypes.Matrix(
                axis_front.x, axis_front.y, axis_front.z, 0,
                axis_up.x, axis_up.y, axis_up.z, 0,
                axis_side.x, axis_side.y, axis_side.z, 0,
                pos.x, pos.y, pos.z, 1
            )
        return tm

    def _get_recommended_pivot_heelfloor(self, pos_foot):
        """
        :param pos_foot: The position of the foot jnt
        :return: The position of the heel pivot
        """
        result = pymel.datatypes.Point(pos_foot)
        result.y = 0
        return result

    def _get_recommended_pivot_front(self, geometries, tm_ref, tm_ref_dir, pos_toes, pos_tip):
        """
        Determine recommended position using ray-cast from the toes.
        If the ray-cast fail, use the last joint position.
        return: The recommended position as a world pymel.datatypes.Vector
        """
        if self.rig.DEFAULT_RIG_FACING_AXIS != constants.Axis.x:
            dir = pymel.datatypes.Point(0, 0, 1) * tm_ref_dir
        else:
            dir = pymel.datatypes.Point(1, 0, 0) * tm_ref_dir
        pos = libRigging.ray_cast_farthest(pos_toes, dir, geometries)
        if not pos:
            cmds.warning("Can't automatically solve FootRoll front pivot, using last joint as reference.")
            pos = pos_tip
        else:
            # Compare our result with the last joint position and take the longuest.
            if self.rig.DEFAULT_RIG_FACING_AXIS != constants.Axis.x:
                pos.z = max(pos.z, pos_tip.z)
            else:
                pos.x = max(pos.x, pos_tip.x)

        # Ensure we are aligned with the reference matrix.
        pos_relative = pos * tm_ref.inverse()
        if self.rig.DEFAULT_RIG_FACING_AXIS != constants.Axis.x:
            pos_relative.x = 0
        else:
            pos_relative.z = 0
        pos_relative.y = 0
        pos = pos_relative * tm_ref
        pos.y = 0

        # HACK : Ensure that the point is size 3 and not 4
        return pymel.datatypes.Point(pos.x, pos.y, pos.z)

    def _get_recommended_pivot_back(self, geometries, tm_ref, tm_ref_dir, pos_toes):
        """
        Determine recommended position using ray-cast from the toes.
        If the ray-cast fail, use the toes position.
        return: The recommended position as a world pymel.datatypes.Vector
        """
        if self.rig.DEFAULT_RIG_FACING_AXIS != constants.Axis.x:
            dir = pymel.datatypes.Point(0, 0, -1) * tm_ref_dir
        else:
            dir = pymel.datatypes.Point(-1, 0, 0) * tm_ref_dir
        pos = libRigging.ray_cast_farthest(pos_toes, dir, geometries)
        if not pos:
            cmds.warning("Can't automatically solve FootRoll back pivot.")
            pos = pos_toes

        # Ensure we are aligned with the reference matrix.
        pos_relative = pos * tm_ref.inverse()
        if self.rig.DEFAULT_RIG_FACING_AXIS != constants.Axis.x:
            pos_relative.x = 0
        else:
            pos_relative.z = 0
        pos_relative.y = 0
        pos = pos_relative * tm_ref
        pos.y = 0

        # HACK : Ensure that the point is size 3 and not 4
        return pymel.datatypes.Point(pos.x, pos.y, pos.z)

    def _get_recommended_pivot_bank(self, geometries, tm_ref, tm_ref_dir, pos_toes, direction=1):
        """
        Determine recommended position using ray-cast from the toes.
        TODO: If the ray-case fail, use a specified default value.
        return: The recommended position as a world pymel.datatypes.Vector
        """
        # Sanity check, ensure that at least one point is in the bounds of geometries.
        # This can prevent rays from being fired from outside a geometry.
        # TODO: Make it more robust.
        filtered_geometries = []
        for geometry in geometries:
            xmin, ymin, zmin, xmax, ymax, zmax = cmds.exactWorldBoundingBox(geometry.__melobject__())
            bound = pymel.datatypes.BoundingBox((xmin, ymin, zmin), (xmax, ymax, zmax))
            if bound.contains(pos_toes):
                filtered_geometries.append(geometry)

        if self.rig.DEFAULT_RIG_FACING_AXIS != constants.Axis.x:
            dir = pymel.datatypes.Point(direction, 0, 0) * tm_ref_dir
        else:
            dir = pymel.datatypes.Point(0, 0, (direction*-1)) * tm_ref_dir
        pos = libRigging.ray_cast_nearest(pos_toes, dir, filtered_geometries)
        if not pos:
            cmds.warning("Can't automatically solve FootRoll bank inn pivot.")
            pos = pos_toes

        pos.y = 0

        return pos

    def _get_ik_ctrl_tms(self):
        """
        Compute the desired rotation for the ik ctrl.
        If the LEGACY_LEG_IK_CTRL_ORIENTATION is set, we'll simply align to the influence.
        :return: A two-size tuple containing the transformation matrix for the ctrl offset and the ctrl itself.
        """
        if self.rig.LEGACY_LEG_IK_CTRL_ORIENTATION:
            return super(LegIk, self)._get_ik_ctrl_tms()

        inf_tm = self.input[self.iCtrlIndex].getMatrix(worldSpace=True)

        # Resolve offset_tm
        offset_tm = pymel.datatypes.Matrix()

        # Resolve ctrl_tm
        axis_dir = constants.Axis.x
        inn_tm_dir = libRigging.get_matrix_axis(inf_tm, axis_dir)
        inn_tm_dir.y = 0  # Ensure the foot ctrl never have pitch values
        # Ensure the ctrl look front
        if inn_tm_dir.z < 0:
            inn_tm_dir = pymel.datatypes.Vector(
                inn_tm_dir.x * -1,
                inn_tm_dir.y * -1,
                inn_tm_dir.z * -1
            )
        inn_tm_upp = pymel.datatypes.Vector(0, 1, 0)

        ctrl_tm = libRigging.get_matrix_from_direction(
            inn_tm_dir,
            inn_tm_upp,
            look_axis=pymel.datatypes.Vector.zAxis,
            upp_axis=pymel.datatypes.Vector.yAxis
        )
        ctrl_tm.translate = inf_tm.translate

        return offset_tm, ctrl_tm

    def build(self, attr_holder=None, constraint_handle=False, setup_softik=True, default_autoroll_threshold=25.0,
              align_footroll_pivot=True, **kwargs):
        """
        Build the LegIk system
        :param attr_holder: The attribute holder object for all the footroll params
        :param constraint_handle: Bool to tell if we constraint the ik handle to the ik ctrl
        :param setup_softik: Bool to know if the leg need a soft ik or not
        :param default_autoroll_threshold: Threshold use to limit the autoroll attribute
        :param align_footroll_pivot: Support the alignment of footroll object in case the foot is not straight
        :param kwargs: More kwargs pass to the superclass
        """
        # Compute ctrl_ik orientation
        super(LegIk, self).build(
            constraint_handle=constraint_handle,
            setup_softik=setup_softik,
            **kwargs
        )

        nomenclature_rig = self.get_nomenclature_rig()

        jnts = self._chain_ik[self.iCtrlIndex:]
        num_jnts = len(jnts)
        if num_jnts == 4:
            jnt_foot, jnt_heel, jnt_toes, jnt_tip = jnts
        elif num_jnts == 3:
            jnt_foot, jnt_toes, jnt_tip = jnts
            jnt_heel = None
        else:
            raise Exception("Unexpected number of joints after the limb. Expected 3 or 4, got {0}".format(num_jnts))

        # Create FootRoll (chain?)
        pos_foot = pymel.datatypes.Point(jnt_foot.getTranslation(space='world'))
        pos_heel = pymel.datatypes.Point(jnt_heel.getTranslation(space='world')) if jnt_heel else None
        pos_toes = pymel.datatypes.Point(jnt_toes.getTranslation(space='world'))
        pos_tip = pymel.datatypes.Point(jnt_tip.getTranslation(space='world'))

        # Resolve pivot locations
        tm_ref = self._get_reference_plane()
        tm_ref_dir = pymel.datatypes.Matrix(  # Used to compute raycast directions
            tm_ref.a00, tm_ref.a01, tm_ref.a02, tm_ref.a03,
            tm_ref.a10, tm_ref.a11, tm_ref.a12, tm_ref.a13,
            tm_ref.a20, tm_ref.a21, tm_ref.a22, tm_ref.a23,
            0, 0, 0, 1
        )

        #
        # Resolve pivot positions
        #
        geometries = self.rig.get_meshes()

        # Resolve pivot inn
        if self.pivot_foot_inn_pos:
            pos_pivot_inn = pymel.datatypes.Point(self.pivot_foot_inn_pos) * tm_ref
        else:
            pos_pivot_inn = self._get_recommended_pivot_bank(geometries, tm_ref, tm_ref_dir, pos_toes, direction=-1)

        # Resolve pivot bank out
        if self.pivot_foot_out_pos:
            pos_pivot_out = pymel.datatypes.Point(self.pivot_foot_out_pos) * tm_ref
        else:
            pos_pivot_out = self._get_recommended_pivot_bank(geometries, tm_ref, tm_ref_dir, pos_toes, direction=1)

        # Resolve pivot Back
        if self.pivot_foot_back_pos:
            pos_pivot_back = pymel.datatypes.Point(self.pivot_foot_back_pos) * tm_ref
        else:
            pos_pivot_back = self._get_recommended_pivot_back(geometries, tm_ref, tm_ref_dir, pos_toes)

        # Set pivot Front
        if self.pivot_foot_front_pos:
            pos_pivot_front = pymel.datatypes.Point(self.pivot_foot_front_pos) * tm_ref
        else:
            pos_pivot_front = self._get_recommended_pivot_front(geometries, tm_ref, tm_ref_dir, pos_toes, pos_tip)

        # Set pivot Ankle
        if self.pivot_toes_ankle_pos:
            pos_pivot_ankle = pymel.datatypes.Point(self.pivot_toes_ankle_pos) * tm_ref
        else:
            pos_pivot_ankle = pos_toes

        # Set pivot Heel floor
        if self.pivot_toes_heel_pos:
            pos_pivot_heel = pymel.datatypes.Point(self.pivot_toes_heel_pos) * tm_ref
        else:
            if jnt_heel:
                pos_pivot_heel = pos_heel
            else:
                pos_pivot_heel = pymel.datatypes.Point(pos_foot)
                pos_pivot_heel.y = 0

        #
        # Build Setup
        #

        root_footRoll = pymel.createNode('transform', name=nomenclature_rig.resolve('footRoll'))

        # Align all pivots to the reference plane
        root_footRoll.setMatrix(tm_ref)

        # Create pivots hierarchy
        self.pivot_toes_heel = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotToesHeel'))
        self.pivot_toes_ankle = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotToesAnkle'))
        self.pivot_foot_ankle = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotFootAnkle'))
        self.pivot_foot_front = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotFootFront'))
        self.pivot_foot_back = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotFootBack'))
        self.pivot_foot_inn = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotFootBankInn'))
        self.pivot_foot_out = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotFootBankOut'))
        self.pivot_foot_heel = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotFootHeel'))
        self.pivot_foot_toes_fk = pymel.spaceLocator(name=nomenclature_rig.resolve('pivotToesFkRoll'))

        # Align all locator with the direction of the foot. Fix problem when foot is not aligned in the world
        if align_footroll_pivot:
            '''
            Ugly way to setup rotate to support maya 2016 before extension 2. Accessing the rotate attribute on a
            transformation matrix crash
            '''
            tmp_container = pymel.spaceLocator()
            tmp_container.setMatrix(tm_ref_dir, worldSpace=True)
            rot = tmp_container.getRotation(space='world')
            self.pivot_foot_ankle.setRotation(rot, space='world')
            self.pivot_foot_inn.setRotation(rot, space='world')
            self.pivot_foot_out.setRotation(rot, space='world')
            self.pivot_foot_back.setRotation(rot, space='world')
            self.pivot_foot_heel.setRotation(rot, space='world')
            self.pivot_foot_front.setRotation(rot, space='world')
            self.pivot_toes_ankle.setRotation(rot, space='world')
            self.pivot_foot_toes_fk.setRotation(rot, space='world')
            self.pivot_toes_heel.setRotation(rot, space='world')
            '''
            self.pivot_foot_ankle.setRotation(tm_ref_dir.rotate, space='world')
            self.pivot_foot_inn.setRotation(tm_ref_dir.rotate, space='world')
            self.pivot_foot_out.setRotation(tm_ref_dir.rotate, space='world')
            self.pivot_foot_back.setRotation(tm_ref_dir.rotate, space='world')
            self.pivot_foot_heel.setRotation(tm_ref_dir.rotate, space='world')
            self.pivot_foot_front.setRotation(tm_ref_dir.rotate, space='world')
            self.pivot_toes_ankle.setRotation(tm_ref_dir.rotate, space='world')
            self.pivot_foot_toes_fk.setRotation(tm_ref_dir.rotate, space='world')
            self.pivot_toes_heel.setRotation(tm_ref_dir.rotate, space='world')
            '''

        chain_footroll = [
            root_footRoll,
            self.pivot_foot_ankle,
            self.pivot_foot_inn,
            self.pivot_foot_out,
            self.pivot_foot_back,
            self.pivot_foot_heel,
            self.pivot_foot_front,
            self.pivot_toes_ankle,
            self.pivot_toes_heel
        ]
        libRigging.create_hyerarchy(chain_footroll)
        chain_footroll[0].setParent(self.grp_rig)
        self.pivot_foot_toes_fk.setParent(self.pivot_foot_heel)

        self.pivot_foot_ankle.setTranslation(pos_pivot_ankle, space='world')
        self.pivot_foot_inn.setTranslation(pos_pivot_inn, space='world')
        self.pivot_foot_out.setTranslation(pos_pivot_out, space='world')
        self.pivot_foot_back.setTranslation(pos_pivot_back, space='world')
        self.pivot_foot_heel.setTranslation(pos_pivot_heel, space='world')
        self.pivot_foot_front.setTranslation(pos_pivot_front, space='world')
        self.pivot_toes_ankle.setTranslation(pos_pivot_ankle, space='world')
        self.pivot_foot_toes_fk.setTranslation(pos_pivot_ankle, space='world')
        self.pivot_toes_heel.setTranslation(pos_pivot_heel, space='world')

        #
        # Create attributes
        #
        attr_holder = self.ctrl_ik
        libAttr.addAttr_separator(attr_holder, 'footRoll', niceName='Foot Roll')
        attr_inn_roll_auto = libAttr.addAttr(attr_holder, longName='rollAuto', k=True)

        # TODO - Tweak value in case we are facing the x axis. Some min and max will be inverted

        # Auto-Roll Threshold
        auto_roll_threshold_default_value = self.attrAutoRollThreshold or default_autoroll_threshold
        self.attrAutoRollThreshold = libAttr.addAttr(
            attr_holder,
            longName=self.AUTOROLL_THRESHOLD_LONGNAME,
            niceName=self.AUTOROLL_THRESHOLD_NICENAME,
            k=True,
            defaultValue=auto_roll_threshold_default_value
        )

        attr_inn_bank = libAttr.addAttr(attr_holder, longName='bank', k=True)
        attr_inn_ankle_roll = libAttr.addAttr(
            attr_holder,
            longName=self.ANKLE_ROTZ_LONGNAME,
            niceName=self.ANKLE_ROTZ_NICENAME,
            k=True, hasMinValue=True, hasMaxValue=True, minValue=-90, maxValue=90
        )
        attr_inn_back_pitch = libAttr.addAttr(
            attr_holder,
            longName=self.BACK_ROTX_LONGNAME,
            niceName=self.BACK_ROTX_NICENAME,
            k=True, hasMinValue=True, hasMaxValue=True, minValue=-90, maxValue=0
        )
        attr_inn_ankle_pitch = libAttr.addAttr(
            attr_holder,
            longName=self.ANKLE_ROTX_LONGNAME,
            niceName=self.ANKLE_ROTX_NICENAME,
            k=True, hasMinValue=True, hasMaxValue=True, minValue=0, maxValue=90
        )
        attr_inn_front_pitch = libAttr.addAttr(
            attr_holder,
            longName=self.FRONT_ROTX_LONGNAME,
            niceName=self.FRONT_ROTX_NICENAME,
            k=True, hasMinValue=True, hasMaxValue=True, minValue=0, maxValue=90
        )
        attr_inn_back_yaw = libAttr.addAttr(
            attr_holder,
            longName=self.BACK_ROTY_LONGNAME,
            niceName=self.BACK_ROTY_NICENAME,
            k=True, hasMinValue=True, hasMaxValue=True, minValue=-90, maxValue=90
        )
        attr_inn_heel_yaw = libAttr.addAttr(
            attr_holder,
            longName=self.HEEL_ROTY_LONGNAME,
            niceName=self.HEEL_ROTY_NICENAME,
            k=True, hasMinValue=True, hasMaxValue=True, minValue=-90, maxValue=90
        )
        attr_inn_toes_yaw = libAttr.addAttr(
            attr_holder,
            longName=self.TOES_ROTY_LONGNAME,
            niceName=self.TOES_ROTY_NICENAME,
            k=True, hasMinValue=True, hasMaxValue=True, minValue=-90, maxValue=90
        )
        attr_inn_front_yaw = libAttr.addAttr(
            attr_holder,
            longName=self.FRONT_ROTY_LONGNAME,
            niceName=self.FRONT_ROTY_NICENAME,
            k=True, hasMinValue=True, hasMaxValue=True, minValue=-90, maxValue=90
        )
        attr_inn_toes_fk_pitch = libAttr.addAttr(
            attr_holder,
            longName=self.TOESFK_ROTX_LONGNAME,
            niceName=self.TOESFK_ROTX_NICENAME,
            k=True, hasMinValue=True, hasMaxValue=True, minValue=-90, maxValue=90
        )

        #
        # Connect attributes
        #

        attr_roll_auto_pos = libRigging.create_utility_node('condition', operation=2, firstTerm=attr_inn_roll_auto,
                                                            secondTerm=0,
                                                            colorIfTrueR=attr_inn_roll_auto,
                                                            colorIfFalseR=0.0).outColorR  # Greater

        attr_roll_auto_f = libRigging.create_utility_node('condition', operation=2,
                                                          firstTerm=attr_inn_roll_auto,
                                                          secondTerm=self.attrAutoRollThreshold,
                                                          colorIfFalseR=0,
                                                          colorIfTrueR=(
                                                              libRigging.create_utility_node('plusMinusAverage',
                                                                                             operation=2,
                                                                                             input1D=[
                                                                                                 attr_inn_roll_auto,
                                                                                                 self.attrAutoRollThreshold]).output1D)
                                                          ).outColorR  # Substract

        attr_roll_auto_b = libRigging.create_utility_node('condition', operation=2, firstTerm=attr_inn_roll_auto,
                                                          secondTerm=0.0,
                                                          colorIfTrueR=0, colorIfFalseR=attr_inn_roll_auto
                                                          ).outColorR  # Greater
        attr_roll_m = libRigging.create_utility_node('addDoubleLinear', input1=attr_roll_auto_pos,
                                                     input2=attr_inn_ankle_pitch).output

        attr_roll_f = libRigging.create_utility_node('addDoubleLinear', input1=attr_roll_auto_f,
                                                     input2=attr_inn_front_pitch).output
        attr_roll_b = libRigging.create_utility_node('addDoubleLinear', input1=attr_roll_auto_b,
                                                     input2=attr_inn_back_pitch).output

        # To keep the same limits, we need to inverse the value when facing the x axis:
        if self.rig.DEFAULT_RIG_FACING_AXIS == constants.Axis.x:
            attr_roll_m = libRigging.create_utility_node('multiplyDivide', operation=1, input1X=-1.0,
                                                         input2X=attr_roll_m).outputX
            attr_roll_f = libRigging.create_utility_node('multiplyDivide', operation=1, input1X=-1.0,
                                                         input2X=attr_roll_f).outputX
            attr_roll_b = libRigging.create_utility_node('multiplyDivide', operation=1, input1X=-1.0,
                                                         input2X=attr_roll_b).outputX

        attr_bank_inn = libRigging.create_utility_node('condition', operation=2,
                                                       firstTerm=attr_inn_bank, secondTerm=0,
                                                       colorIfTrueR=attr_inn_bank,
                                                       colorIfFalseR=0.0
                                                       ).outColorR  # Greater

        attr_bank_out = libRigging.create_utility_node('condition', operation=4,
                                                       firstTerm=attr_inn_bank, secondTerm=0,
                                                       colorIfTrueR=attr_inn_bank,
                                                       colorIfFalseR=0.0).outColorR  # Less

        if self.rig.DEFAULT_RIG_FACING_AXIS != constants.Axis.x:
            pymel.connectAttr(attr_roll_m, self.pivot_toes_ankle.rotateX)
            pymel.connectAttr(attr_roll_f, self.pivot_foot_front.rotateX)
            pymel.connectAttr(attr_roll_b, self.pivot_foot_back.rotateX)
            pymel.connectAttr(attr_bank_inn, self.pivot_foot_inn.rotateZ)
            pymel.connectAttr(attr_bank_out, self.pivot_foot_out.rotateZ)
            pymel.connectAttr(attr_inn_ankle_roll, self.pivot_toes_heel.rotateZ)
            pymel.connectAttr(attr_inn_toes_fk_pitch, self.pivot_foot_toes_fk.rotateX)
        else:
            pymel.connectAttr(attr_roll_m, self.pivot_toes_ankle.rotateZ)
            pymel.connectAttr(attr_roll_f, self.pivot_foot_front.rotateZ)
            pymel.connectAttr(attr_roll_b, self.pivot_foot_back.rotateZ)
            pymel.connectAttr(attr_bank_inn, self.pivot_foot_inn.rotateX)
            pymel.connectAttr(attr_bank_out, self.pivot_foot_out.rotateX)
            pymel.connectAttr(attr_inn_ankle_roll, self.pivot_toes_heel.rotateX)
            pymel.connectAttr(attr_inn_toes_fk_pitch, self.pivot_foot_toes_fk.rotateZ)

        pymel.connectAttr(attr_inn_heel_yaw, self.pivot_foot_heel.rotateY)
        pymel.connectAttr(attr_inn_front_yaw, self.pivot_foot_front.rotateY)
        pymel.connectAttr(attr_inn_back_yaw, self.pivot_foot_back.rotateY)
        pymel.connectAttr(attr_inn_toes_yaw, self.pivot_foot_ankle.rotateY)

        # Create ikHandles and parent them
        # Note that we are directly parenting them so the 'Preserve Child Transform' of the translate tool still work.
        if jnt_heel:
            ikHandle_foot, ikEffector_foot = pymel.ikHandle(startJoint=jnt_foot, endEffector=jnt_heel,
                                                            solver='ikSCsolver')
        else:
            ikHandle_foot, ikEffector_foot = pymel.ikHandle(startJoint=jnt_foot, endEffector=jnt_toes,
                                                            solver='ikSCsolver')
        ikHandle_foot.rename(nomenclature_rig.resolve('ikHandle', 'foot'))
        ikHandle_foot.setParent(self.grp_rig)
        ikHandle_foot.setParent(self.pivot_toes_heel)
        if jnt_heel:
            ikHandle_heel, ikEffector_foot = pymel.ikHandle(startJoint=jnt_heel, endEffector=jnt_toes,
                                                            solver='ikSCsolver')
            ikHandle_heel.rename(nomenclature_rig.resolve('ikHandle', 'heel'))
            ikHandle_heel.setParent(self.grp_rig)
            ikHandle_heel.setParent(self.pivot_foot_front)
        ikHandle_toes, ikEffector_toes = pymel.ikHandle(startJoint=jnt_toes, endEffector=jnt_tip, solver='ikSCsolver')
        ikHandle_toes.rename(nomenclature_rig.resolve('ikHandle', 'toes'))
        ikHandle_toes.setParent(self.grp_rig)
        ikHandle_toes.setParent(self.pivot_foot_toes_fk)

        # Hack: Re-constraint foot ikhandle
        # todo: cleaner!
        pymel.parentConstraint(self.ctrl_ik, root_footRoll, maintainOffset=True)

        # Connect the footroll to the main ikHandle
        # Note that we also need to hijack the softik network.
        fn_can_delete = lambda x: isinstance(x, pymel.nodetypes.Constraint) and \
                                  not isinstance(x, pymel.nodetypes.PoleVectorConstraint)
        pymel.delete(filter(fn_can_delete, self._ik_handle_target.getChildren()))

        if jnt_heel:
            pymel.parentConstraint(self.pivot_toes_heel, self._ik_handle_target, maintainOffset=True)
        else:
            pymel.parentConstraint(self.pivot_toes_ankle, self._ik_handle_target, maintainOffset=True)

        '''
        # Constraint swivel to ctrl_ik
        pymel.parentConstraint(self.ctrl_ik, self.ctrl_swivel,
                               maintainOffset=True)  # TODO: Implement SpaceSwitch
        '''

        # TODO - understand why removing this part give a better result on the position of the footroll loc
        # Handle globalScale
        pymel.connectAttr(self.grp_rig.globalScale, root_footRoll.scaleX)
        pymel.connectAttr(self.grp_rig.globalScale, root_footRoll.scaleY)
        pymel.connectAttr(self.grp_rig.globalScale, root_footRoll.scaleZ)


    def unbuild(self):
        """
        Unbuild the system
        Remember footroll locations in relation with a safe matrix
        The reference matrix is the ankle, maybe we should zero out the y axis.
        :return: Nothing
        """
        # Hold auto-roll threshold
        self.attrAutoRollThreshold = libAttr.hold_attrs(self.attrAutoRollThreshold,
                                                        hold_curve=False)  # only preserve value

        tm_ref_inv = self._get_reference_plane().inverse()

        if self.pivot_foot_heel:
            self.pivot_foot_heel_pos = (self.pivot_foot_heel.getMatrix(worldSpace=True) * tm_ref_inv).translate
        if self.pivot_toes_heel:
            self.pivot_toes_heel_pos = (self.pivot_toes_heel.getMatrix(worldSpace=True) * tm_ref_inv).translate
        if self.pivot_toes_ankle:
            self.pivot_toes_ankle_pos = (self.pivot_toes_ankle.getMatrix(worldSpace=True) * tm_ref_inv).translate
        if self.pivot_foot_front:
            self.pivot_foot_front_pos = (self.pivot_foot_front.getMatrix(worldSpace=True) * tm_ref_inv).translate
        if self.pivot_foot_back:
            self.pivot_foot_back_pos = (self.pivot_foot_back.getMatrix(worldSpace=True) * tm_ref_inv).translate
        if self.pivot_foot_inn:
            self.pivot_foot_inn_pos = (self.pivot_foot_inn.getMatrix(worldSpace=True) * tm_ref_inv).translate
        if self.pivot_foot_out:
            self.pivot_foot_out_pos = (self.pivot_foot_out.getMatrix(worldSpace=True) * tm_ref_inv).translate

        super(LegIk, self).unbuild()

        self.pivot_foot_heel = None
        self.pivot_toes_heel = None
        self.pivot_toes_ankle = None
        self.pivot_foot_front = None
        self.pivot_foot_back = None
        self.pivot_foot_inn = None
        self.pivot_foot_out = None
        self.pivot_foot_ankle = None
        self.pivot_foot_toes_fk = None


class Leg(rigLimb.Limb):
    """
    Basic leg system which use the LegIk class implementation.
    """
    _CLASS_SYS_IK = LegIk

    def validate(self):
        """
        Allow the ui to know if the module is valid to be builded or not
        :return: True or False depending if it pass the building validation
        """
        super(Leg, self).validate()

        num_inputs = len(self.input)
        if num_inputs < 5 or num_inputs > 6:
            raise Exception("Expected between 5 to 6 joints, got {0}".format(num_inputs))

        return True


def register_plugin():
    return Leg
