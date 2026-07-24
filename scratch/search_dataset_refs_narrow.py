import os

search_dir = r"c:\Users\User\OneDrive - Mansoura University - Main\1- Research\Data-Driven"
target_str = "UR5_dataset"
found_files = []

for root, dirs, files in os.walk(search_dir):
    if any(p in root for p in [".gemini", ".git", "AppData", "node_modules", "slprj"]):
        continue
    for file in files:
        if file.endswith((".m", ".txt", ".py", ".md", ".json")):
            full_path = os.path.join(root, file)
            try:
                with open(full_path, "r", encoding="utf-8", errors="ignore") as f:
                    content = f.read()
                    if target_str in content:
                        found_files.append(full_path)
            except Exception as e:
                pass

print("Narrow Search results:")
for path in found_files:
    print(path)
