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
    init_full_path = os.path.join(my_path, "__init__.py")
    requirements_full_path = os.path.join(my_path,"requirements.txt")
    os.makedirs(path, exist_ok=True)
    shutil.copytree(src_full_path,new_src)
    shutil.copy(init_full_path, path)
    shutil.copy(requirements_full_path, path)
    zip_path = os.path.join(my_path,"blender_inertial.zip")
    addon_zip = zipfile.ZipFile(zip_path, mode='w')
    # TODO enable compression
    zipdir("blender_inertial",addon_zip)
    addon_zip.close()
    shutil.rmtree(path)