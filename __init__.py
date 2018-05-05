bl_info = {
    "name": "Inertial-GNSS car data to blender",
    "author": "Federico Bertani",
    "description": "Blender simulation generator from inertial sensor data on car",
    "category": "Object",
}

import os, bpy, urllib.request, sys
from bpy.props import StringProperty, PointerProperty
from bpy.types import PropertyGroup

from .src.__main__ import get_positions_times

bpy.types.Scene.datasetPath = StringProperty(
    name="Dataset path",
    description=":",
    default="",
    maxlen=2056,
)


class InertialBlenderPanel(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = "Tools"
    bl_label = "Inertial-GNSS to blender"

    # bl_context = "object"

    def draw(self, context):
        layout = self.layout

        col = layout.column(align=True)
        col.operator("physicom.load_dataset")
        col.prop(context.scene, "datasetPath")
        col.operator("physicom.animate_object")


class LoadDataset(bpy.types.Operator):
    """Object Cursor Array"""
    # TODO use more standard name
    bl_idname = "physicom.load_dataset"
    bl_label = "Load dataset"
    bl_options = {'REGISTER', 'UNDO'}

    # must be called filepath because fileselect_add() in invoke() assume this
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        context.scene.datasetPath = self.filepath
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class AnimateObject(bpy.types.Operator):
    """Object Cursor Array"""
    # TODO use more standard name
    bl_idname = "physicom.animate_object"
    bl_label = "Animate object"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        scene = context.scene
        fps = bpy.context.scene.render.fps
        obj = scene.objects.active
        positions, times = get_positions_times(scene.datasetPath)
        bpy.context.scene.frame_end = times[-1] * fps
        obj.animation_data_clear()
        # create animation data
        obj.animation_data_create()
        # create a new animation data action
        obj.animation_data.action = bpy.data.actions.new(name="MyAction")
        positions_lenght = positions.shape[1]
        # create f-curve for each axis
        for index in range(3):
            fcurve = obj.animation_data.action.fcurves.new(data_path="location", index=index)
            fcurve.keyframe_points.add(positions_lenght)
            for i in range(0, positions_lenght):
                fcurve.keyframe_points[i].interpolation = 'CONSTANT'
                fcurve.keyframe_points[i].co = times[i] * fps, positions[index,i]
        return {'FINISHED'}


def register():
    # TODO handle permission errors
    addon_path = os.path.dirname(os.path.realpath(__file__))
    blender_path = os.path.dirname(sys.executable)
    # TODO detect version, make more flexible
    blender_python_dir = blender_path + "/2.79/python/bin"
    if not os.path.isfile(blender_python_dir + "/pip"):
        # download get pip
        urllib.request.urlretrieve("https://bootstrap.pypa.io/get-pip.py",
                                   filename=addon_path + "/get_pip.py")
        os.system(blender_python_dir + "/python3.5m " + addon_path + "/get_pip.py")
    else:
        try:
            import scipy
        except ImportError:
            # TODO use something more secure
            os.system(blender_python_dir + "/pip3 install -r " + addon_path + "/requirements.txt")
    # install
    bpy.utils.register_class(LoadDataset)
    bpy.utils.register_class(AnimateObject)
    bpy.utils.register_class(InertialBlenderPanel)
    print("Done!")


def unregister():
    bpy.utils.unregister_class(InertialBlenderPanel)
    bpy.utils.unregister_class(AnimateObject)
    bpy.utils.unregister_class(LoadDataset)


if __name__ == "__main__":
    register()