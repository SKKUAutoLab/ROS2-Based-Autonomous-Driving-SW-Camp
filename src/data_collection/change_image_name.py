import os

def rename_images(folder_path):
    os.chdir(folder_path)
    image_files = [f for f in os.listdir() if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
    
    image_files.sort()
    
    for index, old_name in enumerate(image_files, start=1):
        new_name = f"dataset{index}.png"
        os.rename(old_name, new_name)
        #print(f"Renamed: {old_name} -> {new_name}")
    print("Finish!")

if __name__ == "__main__":
    folder_path = os.path.join(os.path.expanduser('~'), 'ros2_ws/src/load_img/load_img/lib/Collected_Datasets')  # 폴더 경로
    rename_images(folder_path)