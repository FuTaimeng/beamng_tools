import zipfile
import json
import os

def zip_folder(folder_path, zip_path):
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for root, _, files in os.walk(folder_path):
            for file in files:
                file_path = os.path.join(root, file)
                zipf.write(file_path, os.path.relpath(file_path, folder_path))

folders = os.listdir('data')
for folder in folders:
    if folder.startswith('.'):
        continue

    # folder_to_zip = f'data/{folder}'
    # print(folder_to_zip)
    # zip_file_path = f'data_zip/{folder}.zip'
    # zip_folder(folder_to_zip, zip_file_path)

    # if os.path.exists(f'data/{folder}/map2'):
    #     continue
    # cmd = f'python reconstruction.py data/{folder}'
    # print(cmd)
    # os.system(cmd)

    # print(folder)
    # with open(f'data/{folder}/cloud_config.txt', 'r') as f:
    #     config = json.load(f)
    # with open(f'data/{folder}/map2/info.txt', 'r') as f:
    #     info = json.load(f)
    # if 'depth_bit' in info:
    #     continue
    # info['depth_bit'] = config['depth_bit']
    # info['start_frame'] = config['start_frame']
    # info['end_frame'] = config['end_frame']
    # with open(f'data/{folder}/map2/info.txt', 'w') as f:
    #     json.dump(info, f)
    # os.system(f'del data\\{folder}\\cloud_config.txt')
    