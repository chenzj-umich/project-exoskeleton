def modify_constants(file_path, target_string, new_value):
  # Initialize a flag to check if the target string is found
  found = False

  # Read the contents of the file
  with open(file_path, 'r') as file:
    lines = file.readlines()

  # Check each line to find the target string
  for i in range(len(lines)):
    if lines[i].startswith(target_string + " ="):
      lines[i] = f"{target_string} = {new_value}\n"
      found = True
      break

  # If the target string was not found, add it to the end of the file
  if not found:
    lines.append(f"{target_string} = {new_value}\n")

  # Write the changes back to the file
  with open(file_path, 'w') as file:
    file.writelines(lines)