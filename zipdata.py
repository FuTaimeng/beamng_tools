import zipfile
import os

def zip_folder(folder_path, zip_path):
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for root, _, files in os.walk(folder_path):
            for file in files:
                file_path = os.path.join(root, file)
                zipf.write(file_path, os.path.relpath(file_path, folder_path))

folders = os.listdir('data')
for folder in folders[-6:]:
    if folder.startswith('.'):
        continue
    folder_to_zip = f'data/{folder}'
    print(folder_to_zip)
    zip_file_path = f'data_zip/{folder}.zip'
    zip_folder(folder_to_zip, zip_file_path)

    # if os.path.exists(f'data/{folder}/map'):
    #     continue
    # cmd = f'python reconstruction.py {folder}'
    # print(cmd)