import os

search_dir = r"c:\Users\User\OneDrive - Mansoura University - Main"
target_str = "UR5_dataset"
found_files = []

for root, dirs, files in os.walk(search_dir):
    # skip some common system/git folders to speed up
    if any(p in root for p in [".gemini", ".git", "AppData", "node_modules", "slprj"]):
        continue
    for file in files:
        if file.endswith((".m", ".txt", ".py", ".md", ".json", ".docx", ".pdf")):
            full_path = os.path.join(root, file)
            try:
                # For text files
                if not file.endswith((".pdf", ".docx")):
                    with open(full_path, "r", encoding="utf-8", errors="ignore") as f:
                        if target_str in f.read():
                            found_files.append(full_path)
                else:
                    # Just record that we found a file with this name or similar
                    if target_str in file:
                        found_files.append(full_path)
            except Exception as e:
                pass

print("Search results:")
for path in found_files:
    print(path)
