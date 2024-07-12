
# Re-importing the regular expression module, required after the code execution state reset
import re

# Function to read from an input file, remove the first number from each line, renumber the lines starting from 0, and save the output to a new file.
def renumber_lines(input_file_path, output_file_path):
    with open(input_file_path, 'r') as file:
        lines = file.readlines()

    processed_lines = []
    for index, line in enumerate(lines):
        # Remove the first number and any leading/trailing whitespace
        new_line = re.sub(r'^\d+\s+', '', line).strip()
        # Add the new index and the processed line to the list
        processed_lines.append(f"{index} {new_line}")

    # Write the processed data to the output file
    with open(output_file_path, 'w') as file:
        file.write('\n'.join(processed_lines))

    return "Renumbering and saving completed."

# Example file paths (to be replaced with actual file paths by the user)
input_file_path_example = '/home/yf/code/KGE-HAKE-master/data/F2F-V2/entities.dict'  # Placeholder file path
output_file_path_example = '/home/yf/code/KGE-HAKE-master/data/F2F-V2/entities.dict'  # Placeholder output file path

# Note: This function call is an example and won't actually run until real file paths are provided
renumber_lines(input_file_path_example, output_file_path_example)

