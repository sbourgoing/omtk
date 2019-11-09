import pymel.core as pymel
import logging

from omtk.core.classCtrl import BaseCtrl
from omtk.core.classModule import Module
from omtk.libs import libPymel
from omtk.libs import libRigging
from omtk.libs import libSkinning
from omtk.libs import libPython

log = logging.getLogger('omtk')


class CtrlRibbon(BaseCtrl):
    """
    Inherit of base Ctrl to create a specific square shaped controller
    """

    def __createNode__(self, *args, **kwargs):
        node = super(CtrlRibbon, self).__createNode__(*args, **kwargs)
        make = next(iter(node.getShape().create.inputs()), None)
        if make:
            make.degree.set(1)
            make.sections.set(4)
        return node


class Ribbon(Module):
    """
    Generic ribbon setup.
    """

    def __init__(self, *args, **kwargs):
        super(Ribbon, self).__init__(*args, **kwargs)
        self.num_ctrl = 3
        self.ctrls = []
        self.width = 1.0
        self._ribbon_jnts = []
        self._ribbon_shape = None
        self._follicles = []
        self.ribbon_chain_grp = None

    def create_ctrls(self, ctrls=None, no_extremity=False, constraint=True,
                     constraint_rot=True, constraint_scale=True, ctrl_class=CtrlRibbon, **kwargs):
        """
        This function can be used to create controllers on the ribbon joints.
        :param no_extremity: Tell if we want extremity ctrls
        :param constraint: Constraint the ribbon joint over the ctrls ?
        :param constraint_rot: Tell if we constraint the bones on the controllers
        :param ctrl_class: Ctrl class to use to build the ctrl shape
        :return: nothing
        """
        ctrls = ctrls if ctrls else self.ctrls
        nomenclature_anm = self.get_nomenclature_anm()

        # Ensure we have as many ctrls as needed.
        desired_ctrls_count = len(self._ribbon_jnts)
        if no_extremity:
            desired_ctrls_count -= 2
        ctrls = filter(None, ctrls)
        libPython.resize_list(ctrls, desired_ctrls_count)

        real_index = 0
        for i, jnt in enumerate(self._ribbon_jnts):
            if no_extremity and (i == 0 or i == (len(self._ribbon_jnts) - 1)):
                continue
            ctrl = ctrls[real_index] if real_index < len(ctrls) else None
            ctrl_name = nomenclature_anm.resolve(str(real_index + 1).zfill(2))
            # Check if we already have an instance of the ctrl
            if not isinstance(ctrl, ctrl_class):
                ctrl = ctrl_class()
                ctrls[real_index] = ctrl
            ctrl.build(name=ctrl_name, size=self.width, **kwargs)
            ctrl.setMatrix(jnt.getMatrix(worldSpace=True), worldSpace=True)
            ctrl.setParent(self.grp_anm)

            if constraint:
                if constraint_rot:
                    pymel.parentConstraint(ctrl, jnt, mo=True)
                else:
                    pymel.pointConstraint(ctrl, jnt, mo=True)
                if constraint_scale:
                    pymel.connectAttr(ctrl.scaleX, jnt.scaleX)
                    pymel.connectAttr(ctrl.scaleY, jnt.scaleY)
                    pymel.connectAttr(ctrl.scaleZ, jnt.scaleZ)

            real_index += 1

        return ctrls

    def attach_to_plane(self, constraint_rot=True, ribbon_name='ribbon'):
        """
        Create follicle attached to the place for each input joint
        :param constraint_rot: Are the joints will be constraint in rotation on the follicle
        :return: Nothing
        """
        nomenclature_rig = self.get_nomenclature_rig()
        fol_v = 0.5  # Always in the center

        # split_value = 1.0 / (len(self.chain_jnt) - 1)

        for i, jnt in enumerate(self.chain_jnt):
            # fol_u = split_value * i
            # TODO: Validate that we don't need to inverse the rotation separately.
            jnt_pos = jnt.getMatrix(worldSpace=True).translate
            pos, fol_u, fol_v = libRigging.get_closest_point_on_surface(self._ribbon_shape, jnt_pos)
            fol_name = nomenclature_rig.resolve("{1}Follicle{0:02d}".format(i, ribbon_name))
            fol_shape = libRigging.create_follicle2(self._ribbon_shape, u=fol_u, v=fol_v)
            fol = fol_shape.getParent()
            fol.rename(fol_name)
            if constraint_rot:
                pymel.parentConstraint(fol, jnt, mo=True)
            else:
                pymel.pointConstraint(fol, jnt, mo=True)

            self._follicles.append(fol)

    def build(self, no_subdiv=False, num_ctrl=None, degree=3, create_ctrl=True, rot_fol=True,
              attach_input=True, ribbon_name='ribbon', assign_bind_pre_matrix=False, chain_ribbon_jnt=False,
              *args, **kwargs):
        """
        Build the ribbon system

        :param no_subdiv: Tell if the ribbon need to have subdivision in the place or not
        :param num_ctrl: The number of ctrl needed to ctrl the ribbon, will influence the number of joint used to deform
                         the nurbs
        :param degree: Degree that will be used to create the nurbs
        :param create_ctrl: Tell the system to create the ctrl or not
        :param rot_fol: Constraint the input joint rotation to the follicle use to attach them
        :param attach_input: Tell the system to attach the input joint to the nurbs surface or not
        :param ribbon_name: String used to defined the ribbon system
        :param assign_bind_pre_matrix: Assign the bind pre-matrix to the inverse parent of the joint to only
                                       deform local
        :param chain_ribbon_jnt: Are the ribbon joint will be chained or not
        :param args: Additional arguments (Could be module args)
        :param kwargs: Additional keywords arguments (Could be module kwargs)
        :return:
        """
        super(Ribbon, self).build(create_grp_anm=create_ctrl, *args, **kwargs)
        if num_ctrl is not None:
            self.num_ctrl = num_ctrl

        nomenclature_rig = self.get_nomenclature_rig()

        # Create the plane and align it with the selected bones
        plane_tran = next(
            (input for input in self.input if libPymel.isinstance_of_shape(input, pymel.nodetypes.NurbsSurface)), None)
        if plane_tran is None:
            plane_name = nomenclature_rig.resolve("{0}_plane".format(ribbon_name))
            if no_subdiv:  # We don't want any subdivision in the plane, so use only 2 bones to create it
                no_subdiv_degree = 2
                if degree < 2:
                    no_subdiv_degree = degree
                plane_tran = libRigging.create_nurbs_plane_from_joints([self.chain_jnt[0], self.chain_jnt[-1]],
                                                                       degree=no_subdiv_degree, width=self.width)
            else:
                plane_tran = libRigging.create_nurbs_plane_from_joints(self.chain_jnt, degree=degree, width=self.width)
            plane_tran.rename(plane_name)
            plane_tran.setParent(self.grp_rig)
        self._ribbon_shape = plane_tran.getShape()

        # Create the follicule needed for the system on the skinned bones
        if attach_input:
            self.attach_to_plane(rot_fol, ribbon_name)
            # TODO : Support aim constraint for bones instead of follicle rotation?

            follicles_grp = pymel.createNode("transform")
            follicle_grp_name = nomenclature_rig.resolve("{0}_input_follicle".format(ribbon_name))
            follicles_grp.rename(follicle_grp_name)
            follicles_grp.setParent(self.grp_rig)
            for n in self._follicles:
                n.setParent(follicles_grp)

        # Create the joints that will drive the ribbon.
        # TODO: Support other shapes than straight lines...
        self._ribbon_jnts = libRigging.create_chain_between_objects(
            self.chain_jnt.start, self.chain_jnt.end, self.num_ctrl, parented=chain_ribbon_jnt)

        # Group all the joints
        ribbon_chain_grp_name = nomenclature_rig.resolve("{0}_chain".format(ribbon_name))
        self.ribbon_chain_grp = pymel.createNode('transform', name=ribbon_chain_grp_name, parent=self.grp_rig)
        align_chain = True if len(self.chain_jnt) == len(self._ribbon_jnts) else False
        for i, jnt in enumerate(self._ribbon_jnts):
            # Align the ribbon joints with the real joint to have a better rotation ctrl
            ribbon_jnt_name = nomenclature_rig.resolve('{1}Jnt{0:02d}'.format(i, ribbon_name))
            jnt.rename(ribbon_jnt_name)
            # If we keep the chain, just parent the first
            if not chain_ribbon_jnt:
                jnt.setParent(self.ribbon_chain_grp)
            else:
                if i == 0:
                    jnt.setParent(self.ribbon_chain_grp)
                else:
                    # Ensure all joint have the same orientation
                    jnt.jointOrient.set([0, 0, 0])
            if align_chain:
                matrix = self.chain_jnt[i].getMatrix(worldSpace=True)
                jnt.setMatrix(matrix, worldSpace=True)
            # Create a zero grp to prevent problem when assigning bind pre matrix
            if assign_bind_pre_matrix:
                zero_grp_name = nomenclature_rig.resolve('{1}Jnt_zero{0:02d}'.format(i, ribbon_name))
                zero_grp = pymel.createNode('transform', name=zero_grp_name, parent=self.grp_rig)
                zero_grp.setParent(jnt.getParent())
                zero_grp.setMatrix(jnt.getMatrix(worldSpace=True), worldSpace=True)
                jnt.setParent(zero_grp)
                # Freeze transform the joint and remove any joint orient to have a joint at 0
                pymel.makeIdentity(jnt, r=True)
                jnt.jointOrient.set([0, 0, 0])

        # TODO - Improve skinning smoothing by setting manually the skin...
        skin = pymel.skinCluster(list(self._ribbon_jnts), plane_tran, dr=1.0, mi=2.0, omi=True)
        if assign_bind_pre_matrix:
            libSkinning.assign_bind_pre_matrix(skin)
        try:
            libSkinning.assign_weights_from_segments(self._ribbon_shape, self._ribbon_jnts, dropoff=1.0)
        except:
            log.warning("Could not assign automatibly skin weights for ribbon {0}".format(ribbon_name))
            pass

        # Create the ctrls that will drive the joints that will drive the ribbon.
        if create_ctrl:
            self.ctrls = self.create_ctrls(**kwargs)

            # Global uniform scale support
            self.globalScale.connect(self.ribbon_chain_grp.scaleX)
            self.globalScale.connect(self.ribbon_chain_grp.scaleY)
            self.globalScale.connect(self.ribbon_chain_grp.scaleZ)

    def unbuild(self):
        super(Ribbon, self).unbuild()

        self.ctrls = []


def register_plugin():
    return Ribbon
