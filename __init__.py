bl_info = {
    "name": "Inertial-GNSS car data to blender",
    "author": "Federico Bertani",
    "description": "Blender simulation generator from inertial sensor data on car",
    "category": "Object",
}

import bpy
import os
import sys
import urllib.request
from pathlib import Path
from subprocess import call

from bpy.props import StringProperty

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
        sys.path.append(str(Path(__file__).parent))
        from src.create_trajectory_file import get_positions_times
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


def call_system_command(command):
    try:
        retcode = call(command, shell=True)
        if retcode < 0:
            print("Child was terminated by signal", -retcode, file=sys.stderr)
        else:
            print("Child returned", retcode, file=sys.stderr)
    except OSError as e:
        print("Execution failed:", e, file=sys.stderr)

def register():
    # TODO handle permission errors
    addon_path = str(Path(__file__).parent)
    print("Addon path " + addon_path)
    blender_path = str(Path(sys.executable).parent)
    print("Blender path " + blender_path)
    # TODO detect version, make more flexible
    #_, dirs, _ = next(os.walk(blender_path))
    #blender_version_dir = dirs[0]
    blender_version_dir = "2.79"
    blender_python_dir = os.path.join(blender_path, blender_version_dir, "python")
    posix_pip_location = os.path.join(blender_python_dir, "bin", "pip")
    windows_pip_location = os.path.join(blender_python_dir, "scripts", "pip3.exe")
    if not (os.path.exists(posix_pip_location) or os.path.exists(windows_pip_location)):
        print("Downloading pip")
        # download get pip
        pip_download_location = os.path.join(addon_path, "get_pip.py")
        urllib.request.urlretrieve("https://bootstrap.pypa.io/get-pip.py",
                                   filename=pip_download_location)
        python_bin = os.path.join(blender_python_dir,"bin")
        if (os.path.exists(os.path.join(python_bin, "python3.5m"))):
            python_interpreter = os.path.join(python_bin, "python3.5m")
        elif os.path.exists(os.path.join(python_bin, "python.exe")):
            python_interpreter = os.path.join(python_bin, "python.exe")
        command = r'"{}" "{}"'.format(python_interpreter,pip_download_location)
        print("Command: " + command)
        call_system_command(command)
    try:
        import scipy, quaternion
    except ImportError:
        print("Installing packets")
        if (os.path.exists(posix_pip_location)):
            command = r'"{}" install -r "{}"'.format(posix_pip_location,os.path.join(addon_path,"requirements.txt"))
        elif os.path.exists(windows_pip_location):
            command = r'"{}" install -r "{}"'.format(windows_pip_location,os.path.join(addon_path,"requirements.txt"))
        call_system_command(command)
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
