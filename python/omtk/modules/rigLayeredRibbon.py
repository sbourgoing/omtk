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

        self._parented_layer_ctrl_grp = None
        self._noparented_layer_ctrl_grp = None

        # Init all other num ctrl to the min ctrl a ribbon can have
        for i in xrange(0, self.MAX_LAYER):
            if i >= self.num_layer:
                self.num_ctrl_by_layer.append(2)
                self.is_chained_ctrl_by_layer.append(False)

    def attach_to_ribbon(self, ribbon_shape, const_node, constraint_rot=True):
        """
        Create follicle attached to desired ribbon shape
        :param ribbon_shape: The ribbon shape use to attach the ctrl on
        :param const_node: List of nodes to be attach to ribbon
        :param constraint_rot: Are the joints will be constraint in rotation on the follicle
        :return: Nothing
        """
        nomenclature_rig = self.get_nomenclature_rig()
        fol_v = 0.5  # Always in the center

        # split_value = 1.0 / (len(self.chain_jnt) - 1)

        new_fols = []

        for i, node in enumerate(const_node):
            # fol_u = split_value * i
            # TODO: Validate that we don't need to inverse the rotation separately.
            jnt_pos = node.getMatrix(worldSpace=True).translate
            pos, fol_u, fol_v = libRigging.get_closest_point_on_surface(ribbon_shape, jnt_pos)
            fol_name = nomenclature_rig.resolve("{1}Follicle{0:02d}".format(i, 'ribbon'))
            fol_shape = libRigging.create_follicle2(ribbon_shape, u=fol_u, v=fol_v)
            fol = fol_shape.getParent()
            fol.rename(fol_name)
            if constraint_rot:
                pymel.parentConstraint(fol, node, mo=True)
            else:
                pymel.pointConstraint(fol, node, mo=True)

            new_fols.append(fol)

        return new_fols

    def build(self, *args, **kwargs):
        super(LayeredRibbon, self).build(create_grp_anm=True, parent=False, *args, **kwargs)

        nomenclature_rig = self.get_nomenclature_rig()
        nomenclature_anm = self.get_nomenclature_anm()
        ctrl_suffix = self.rig.nomenclature.type_anm

        # If after a rebuild the number of ctrl changes (because of a layer number change or num of ctrl change),
        # delete the old ctrl, else keep them for rebuild
        total_ctrls = sum(self.num_ctrl_by_layer[0:self.num_layer])
        same_num_ctrl = len(self.ctrls) == total_ctrls

        # TODO - Support a check to know the number of ctrl have changed in the different layer. If yes, delete ctrls

        if self.ctrls and not same_num_ctrl:
            # pymel.delete(self.ctrls)
            self.ctrls = []

        # This group will be parented to the master anm ctrl
        parented_layer_ctrl_grp_name = nomenclature_anm.resolve("nochain_layers")
        self._parented_layer_ctrl_grp = pymel.createNode('transform', name=parented_layer_ctrl_grp_name,
                                                         parent=self.grp_anm)
        # This group will keep chained ribbon layer (except the first if it's the case) and will not be parented
        # to the master anm group
        notparented_layer_ctrl_grp_name = nomenclature_anm.resolve("chained_layers")
        self._notparented_layer_ctrl_grp = pymel.createNode('transform', name=notparented_layer_ctrl_grp_name,
                                                            parent=self.grp_anm)

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
            pymel.parentConstraint(self._parented_layer_ctrl_grp, sys_ribbon.ribbon_chain_grp)
            pymel.scaleConstraint(self._parented_layer_ctrl_grp, sys_ribbon.ribbon_chain_grp)

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
            master_parent = self._parented_layer_ctrl_grp
            if self.is_chained_ctrl_by_layer[i] and i != 0:
                master_parent = self._notparented_layer_ctrl_grp

            master_layer_ctrl_grp_name = nomenclature_anm.resolve("layer{0:02d}".format(i))
            master_layer_ctrl_grp = pymel.createNode('transform', name=master_layer_ctrl_grp_name,
                                                     parent=master_parent)

            previous_ctrl = None
            for j, ctrl in enumerate(rib_ctrls):
                libAttr.unlock_trs(ctrl.offset)
                # Reparent ctrl if needed
                # TODO - Remove this part, this is a test. Need to be implemented a way the user can choose
                if self.is_chained_ctrl_by_layer[i]:
                    if previous_ctrl is not None:
                        ctrl.setParent(previous_ctrl)
                    else:
                        ctrl.setParent(master_layer_ctrl_grp)
                    previous_ctrl = ctrl
                else:
                    ctrl.setParent(master_layer_ctrl_grp)

            # Constraint the ctrls on the previous layer ribbon shape if needed
            if i != 0:
                if not self.is_chained_ctrl_by_layer[i]:
                    all_offset = [ctrl.offset for ctrl in rib_ctrls]
                    all_fol = self.attach_to_ribbon(previous_ribbon._ribbon_shape, all_offset)
                    layer_fol_grp_name = nomenclature_rig.resolve("{1}_fol_layer{0:02d}".format(i, ctrl_suffix))
                    layer_fol_grp = pymel.createNode('transform', name=layer_fol_grp_name, parent=rib.grp_rig)
                    for fol in all_fol:
                        fol.setParent(layer_fol_grp)

                    # Now setup the ctrl/ribbon joint relation
                    for ctrl, rib_jnt in zip(rib_ctrls, rib._ribbon_jnts):
                        # Ribbon joint will have a zero parent setup in the ribbon creation since
                        # bind pre matrix is true
                        libAttr.connect_transform_attrs(ctrl.offset, rib_jnt.getParent(), sx=False, sy=False, sz=False)
                        libAttr.connect_transform_attrs(ctrl, rib_jnt)
                else:
                    # Setup is different if the user want a chained layer

                    # First, create an offset hierarchy to support
                    layer_ctrl_offset_name = nomenclature_anm.resolve("{1}_offset_layer{0:02d}"
                                                                      .format(i, ctrl_suffix))
                    layer_ctrl_offset_grp = pymel.createNode('transform', name=layer_ctrl_offset_name,
                                                                  parent=rib.grp_rig)
                    parent_offset = None
                    ctrl_offset_list = []
                    for j, ctrl in enumerate(rib_ctrls):
                        ctrl_offset_wm = ctrl.offset.getMatrix(worldSpace=True)
                        ctrl_offset_name = nomenclature_anm.resolve("{1}_offset{2:02d}_layer{0:02d}"
                                                                    .format(i, ctrl_suffix, j))
                        # Create the hierarchy based on the ctrl offset
                        if not parent_offset:
                            ctrl_offset = pymel.createNode('transform', name=ctrl_offset_name,
                                                           parent=layer_ctrl_offset_grp)
                        else:
                            ctrl_offset = pymel.createNode('transform', name=ctrl_offset_name,
                                                           parent=parent_offset)

                        ctrl_offset.setMatrix(ctrl_offset_wm, worldSpace=True)
                        parent_offset = ctrl_offset
                        ctrl_offset_list.append(ctrl_offset)

                    # Attach the offset to the previous ribbon
                    offset_fol_list = self.attach_to_ribbon(previous_ribbon._ribbon_shape, ctrl_offset_list)
                    layer_fol_grp_name = nomenclature_rig.resolve("{1}_offset)fol_layer{0:02d}".format(i, ctrl_suffix))
                    layer_fol_grp = pymel.createNode('transform', name=layer_fol_grp_name, parent=rib.grp_rig)
                    for fol in offset_fol_list:
                        fol.setParent(layer_fol_grp)

                    # Connect the offset directly to the ctrl offset node. Also parent constraint the ribbon joint
                    # offset to the same offset
                    for offset, ctrl, ribbon_jnt in zip(ctrl_offset_list, rib_ctrls, rib._ribbon_jnts):
                        libAttr.connect_transform_attrs(offset, ctrl.offset, sx=False, sy=False, sz=False)
                        pymel.parentConstraint(offset, ribbon_jnt.getParent())

                        # Create the matrice node network to connect the ribbon joint to the ctrl

                        # 1. Compute the offset between ctrl and it's extra offset
                        ctrl_offset_wm = ctrl.offset.worldMatrix
                        ctrl_extra_offset_wm = offset.worldInverseMatrix

                        util_mult_ctrl_offset = libRigging.create_utility_node(
                            'multMatrix',
                        )

                        pymel.connectAttr(ctrl_offset_wm, util_mult_ctrl_offset.matrixIn[0])
                        pymel.connectAttr(ctrl_extra_offset_wm, util_mult_ctrl_offset.matrixIn[1])

                        # 2. Convert the previous offset in the space of the ctrl

                        util_mult_ctrl_space = libRigging.create_utility_node(
                            'multMatrix',
                        )

                        pymel.connectAttr(ctrl.matrix, util_mult_ctrl_space.matrixIn[0])
                        pymel.connectAttr(util_mult_ctrl_offset.matrixSum, util_mult_ctrl_space.matrixIn[1])

                        # 3. Decompose the constructed matrix and affect it to the ribbon joint

                        attr_tm = util_mult_ctrl_space.matrixSum

                        util_decompose = libRigging.create_utility_node(
                            'decomposeMatrix',
                            inputMatrix=attr_tm
                        )
                        pymel.connectAttr(util_decompose.outputTranslate, ribbon_jnt.translate)
                        pymel.connectAttr(util_decompose.outputRotate, ribbon_jnt.rotate)
                        # pymel.connectAttr(util_decompose.outputScale, node.scale)





            previous_ribbon = rib

    def parent_to(self, parent):
        """
        Parent the system to a specific object.
        """

        # Chained layered ctrl don't need to be parented, but the no chained yes
        pymel.parentConstraint(parent, self._parented_layer_ctrl_grp, maintainOffset=True)
        pymel.scaleConstraint(parent, self._parented_layer_ctrl_grp, maintainOffset=True)

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
