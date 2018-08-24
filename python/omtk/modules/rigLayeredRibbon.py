import pymel.core as pymel
from omtk.core.classCtrl import BaseCtrl
from omtk.core.classModule import Module
from omtk.modules.rigRibbon import Ribbon
from omtk.libs import libCtrlShapes
from omtk.libs import libPymel
from omtk.libs import libRigging
from omtk.libs import libSkinning
from omtk.libs import libPython
from omtk.libs import libAttr


class CtrlLayer01(BaseCtrl):
    """
    Inherit of base Ctrl to create a specific square shaped controller
    """

    def __createNode__(self, *args, **kwargs):
        if 'size' in kwargs:
            kwargs['size'] = kwargs['size'] * 1.75
        node = super(CtrlLayer01, self).__createNode__(*args, **kwargs)
        node.drawOverride.overrideEnabled.set(1)
        node.drawOverride.overrideColor.set(24)
        make = next(iter(node.getShape().create.inputs()), None)
        if make:
            make.degree.set(3)
            make.sections.set(8)
        return node


class CtrlLayer02(BaseCtrl):
    def __createNode__(self, **kwargs):
        if 'size' in kwargs:
            kwargs['size'] = kwargs['size'] * 1.4
        node = libCtrlShapes.create_shape_cross(**kwargs)
        node.drawOverride.overrideEnabled.set(1)
        node.drawOverride.overrideColor.set(25)
        return node


class CtrlLayer03(BaseCtrl):
    """
    Inherit of base Ctrl to create a specific square shaped controller
    """

    def __createNode__(self, *args, **kwargs):
        if 'size' in kwargs:
            kwargs['size'] = kwargs['size'] * 1.3
        node = super(CtrlLayer03, self).__createNode__(*args, **kwargs)
        node.drawOverride.overrideEnabled.set(1)
        node.drawOverride.overrideColor.set(26)
        make = next(iter(node.getShape().create.inputs()), None)
        if make:
            make.degree.set(1)
            make.sections.set(5)
        return node


class CtrlLayer04(BaseCtrl):
    """
    Inherit of base Ctrl to create a specific square shaped controller
    """

    def __createNode__(self, *args, **kwargs):
        if 'size' in kwargs:
            kwargs['size'] = kwargs['size'] * 1.2
        node = super(CtrlLayer04, self).__createNode__(*args, **kwargs)
        node.drawOverride.overrideEnabled.set(1)
        node.drawOverride.overrideColor.set(27)
        make = next(iter(node.getShape().create.inputs()), None)
        if make:
            make.degree.set(1)
            make.sections.set(3)
        return node


class LayeredRibbon(Module):
    """
    Multiple ribbon will be layered one over each other to be able to have different deformation stacked. As an example,
    you can have a Ik - Fk - Tweak system which will give you the possibility to shape with fk first, displace and
    modify with a ribbon ik and finally tweak the shape again with additional tweak ctrls
    """

    # TODO - Correctly support scalling

    # At the moment, this is a bit complex to tweak the number of ctrls wanted to the number of layer. So we will
    # at the moment allocated the space for a maximum of 10 layer.
    # TODO - Implement a property tweak windows to facilitate this
    MAX_LAYER = 10
    # List containing the ctrl class type used by the different layers
    LAYER_CTRL_CLS = [CtrlLayer01, CtrlLayer02, CtrlLayer03, CtrlLayer04, CtrlLayer04,
                      CtrlLayer04, CtrlLayer04, CtrlLayer04, CtrlLayer04, CtrlLayer04]

    def __init__(self, *args, **kwargs):
        super(LayeredRibbon, self).__init__(*args, **kwargs)
        self._layered_ribbon = []  # List of all ribbon used in the system
        # List of the different follicle used to drive the ctrls of the second and more layers
        self._ctrls_follicles = []
        self.num_layer = 3
        self.num_ctrl_by_layer = [len(self.jnts), 2, len(self.jnts)/2]  # Base setup is Fk - Ik - Tweak
        self.is_chained_ctrl_by_layer = [True, False, False]
        self.ctrls = []  # List to keep all ctrls, this will need to be hashed to get the good ctrls for each ribbon
        self.ribbon_width = 1.0

        # Init all other num ctrl to the min ctrl a ribbon can have
        for i in xrange(0, self.MAX_LAYER):
            if i >= self.num_layer:
                self.num_ctrl_by_layer.append(2)
                self.is_chained_ctrl_by_layer.append(False)

    def attach_to_ribbon(self, ribbon_shape, ctrls, constraint_rot=True):
        """
        Create follicle attached to desired ribbon shape
        :param ribbon_shape: The ribbon shape use to attach the ctrl on
        :param ctrls: The ctrls to be attach to ribbon
        :param constraint_rot: Are the joints will be constraint in rotation on the follicle
        :return: Nothing
        """
        nomenclature_rig = self.get_nomenclature_rig()
        fol_v = 0.5  # Always in the center

        # split_value = 1.0 / (len(self.chain_jnt) - 1)

        new_fols = []

        for i, ctrl in enumerate(ctrls):
            # fol_u = split_value * i
            # TODO: Validate that we don't need to inverse the rotation separately.
            jnt_pos = ctrl.getMatrix(worldSpace=True).translate
            pos, fol_u, fol_v = libRigging.get_closest_point_on_surface(ribbon_shape, jnt_pos)
            fol_name = nomenclature_rig.resolve("{1}Follicle{0:02d}".format(i, 'ribbon'))
            fol_shape = libRigging.create_follicle2(ribbon_shape, u=fol_u, v=fol_v)
            fol = fol_shape.getParent()
            fol.rename(fol_name)
            if constraint_rot:
                pymel.parentConstraint(fol, ctrl.offset, mo=True)
            else:
                pymel.pointConstraint(fol, ctrl.offset, mo=True)

            new_fols.append(fol)

        return new_fols

    def build(self, *args, **kwargs):
        super(LayeredRibbon, self).build(create_grp_anm=True, *args, **kwargs)

        nomenclature_rig = self.get_nomenclature_rig()

        # If after a rebuild the number of ctrl changes (because of a layer number change or num of ctrl change),
        # delete the old ctrl, else keep them for rebuild
        total_ctrls = sum(self.num_ctrl_by_layer[0:self.num_layer])
        same_num_ctrl = len(self.ctrls) == total_ctrls

        # TODO - Support a check to know the number of ctrl have changed in the different layer. If yes, delete ctrls

        if self.ctrls and not same_num_ctrl:
            # pymel.delete(self.ctrls)
            self.ctrls = []

        # Build all the ribbon system needed
        for i in xrange(self.num_layer):
            num_layer_ctrl = self.num_ctrl_by_layer[i]

            cur_ribbon = None
            if i < len(self._layered_ribbon):
                cur_ribbon = self._layered_ribbon[i]

            sys_ribbon = self.init_module(Ribbon, cur_ribbon, inputs=self.jnts, suffix='layer{0:02d}'.format(i))
            sys_ribbon.width = self.ribbon_width

            # Some params will change depending of the ribbon layer position
            if i == 0:
                attach_input = False
                bind_pre_matrix = False
            elif i == (self.num_layer - 1):
                attach_input = True
                bind_pre_matrix = True
            else:
                attach_input = False
                bind_pre_matrix = True

            sys_ribbon.build(create_ctrl=False, degree=3, num_ctrl=num_layer_ctrl, no_subdiv=False, rot_fol=True,
                             attach_input=attach_input, assign_bind_pre_matrix=bind_pre_matrix,
                             chain_ribbon_jnt=self.is_chained_ctrl_by_layer[i])

            if not cur_ribbon:
                self._layered_ribbon.append(sys_ribbon)

            sys_ribbon.grp_rig.setParent(self.grp_rig)
            pymel.parentConstraint(self.grp_anm, sys_ribbon.ribbon_chain_grp)

        # Setup ribbon shape deformer stack using blendshapes
        for i, rib in enumerate(self._layered_ribbon):
            if i == len(self._layered_ribbon) - 1:
                break
            pymel.blendShape(rib._ribbon_shape, self._layered_ribbon[i + 1]._ribbon_shape, en=1.0, w=[(0, 1.0)],
                             foc=True)

        # Setup the different ctrl needed
        cur_ctrl_idx = 0
        previous_ribbon = None
        skip_reuse = False
        for i, rib in enumerate(self._layered_ribbon):
            existing_ctrls = None
            if self.ctrls and not skip_reuse:
                rib_num_jnts = len(rib._ribbon_jnts)
                existing_ctrls = self.ctrls[cur_ctrl_idx:cur_ctrl_idx + rib_num_jnts]
                cur_ctrl_idx = cur_ctrl_idx + rib_num_jnts
            # first ribbon ctrl will simply drive the joint with parent contraint
            if i == 0:
                rib_ctrls = rib.create_ctrls(constraint=True, ctrl_class=self.LAYER_CTRL_CLS[i],
                                             ctrls=existing_ctrls, constraint_scale=False)
                if not existing_ctrls:
                    skip_reuse = True
            else:
                rib_ctrls = rib.create_ctrls(constraint=False, ctrl_class=self.LAYER_CTRL_CLS[i],
                                             ctrls=existing_ctrls, constraint_scale=False)
            if not existing_ctrls:
                self.ctrls.extend(rib_ctrls)

            # Create a group to keep all the ctrls
            layer_ctrl_grp_name = nomenclature_rig.resolve("layer{0:02d}".format(i))
            layer_ctrl_grp = pymel.createNode('transform', name=layer_ctrl_grp_name, parent=self.grp_anm)
            previous_ctrl = None
            for j, ctrl in enumerate(rib_ctrls):
                libAttr.unlock_trs(ctrl.offset)
                # Reparent ctrl if needed
                # TODO - Remove this part, this is a test. Need to be implemented a way the user can choose
                if self.is_chained_ctrl_by_layer[i]:
                    if previous_ctrl is not None:
                        ctrl.setParent(previous_ctrl)
                    else:
                        ctrl.setParent(layer_ctrl_grp)
                    previous_ctrl = ctrl
                else:
                    ctrl.setParent(layer_ctrl_grp)

            # Constraint the ctrls on the previous layer ribbon shape if needed
            if i != 0:
                all_fol = self.attach_to_ribbon(previous_ribbon._ribbon_shape, rib_ctrls)
                layer_fol_grp_name = nomenclature_rig.resolve("ctrl_fol_layer{0:02d}".format(i))
                layer_fol_grp = pymel.createNode('transform', name=layer_fol_grp_name, parent=rib.grp_rig)
                for fol in all_fol:
                    fol.setParent(layer_fol_grp)

                # Now setup the ctrl/ribbon joint relation
                for ctrl, rib_jnt in zip(rib_ctrls, rib._ribbon_jnts):
                    # Ribbon joint will have a zero parent setup in the ribbon creation since bind pre matrix is true
                    libAttr.connect_transform_attrs(ctrl.offset, rib_jnt.getParent(), sx=False, sy=False, sz=False)
                    libAttr.connect_transform_attrs(ctrl, rib_jnt)

            previous_ribbon = rib

    def unbuild(self):
        super(LayeredRibbon, self).unbuild()

        for rib_sys in self._layered_ribbon:
            rib_sys.unbuild()

    def validate(self):
        """
        Allow the ui to know if the module is valid to be builded or not
        :return: True or False depending if it pass the building validation
        """
        super(LayeredRibbon, self).validate()

        if self.num_layer > self.MAX_LAYER:
            raise Exception("Maximum Layer supported is {0}".format(self.MAX_LAYER))

        if self.num_layer < 2:
            raise Exception("Minimum Layer supported is {0}".format(2))

        return True


def register_plugin():
    return LayeredRibbon
