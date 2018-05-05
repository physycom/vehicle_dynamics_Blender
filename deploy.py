#!/usr/bin/env python3

def zipdir(path, ziph):
    # ziph is zipfile handle
    for root, dirs, files in os.walk(path):
        for file in files:
            ziph.write(os.path.join(root, file))

if __name__=="__main__":
    import os, zipfile, shutil

    my_path = os.path.abspath(os.path.dirname(__file__))
    path = os.path.join(my_path, "blender_inertial")
    src_full_path = os.path.join(my_path, "src")
    new_src = os.path.join(path,"src")
    init_full_paht = os.path.join(my_path, "__init__.py")
    new_init = os.path.join(path,"__init__.py")
    os.makedirs(path, exist_ok=True)
    shutil.copytree(src_full_path,new_src)
    shutil.copy(init_full_paht,new_init)
    zip_path = os.path.join(my_path,"blender_inertial.zip")
    addon_zip = zipfile.ZipFile(zip_path, mode='w')
    zipdir("blender_inertial",addon_zip)
    addon_zip.close()
    shutil.rmtree(path)