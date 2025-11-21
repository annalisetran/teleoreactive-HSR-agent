import os

def process_files_in_directory():
    # Get list of all files in the current directory
    files = [f for f in os.listdir('.') if os.path.isfile(f)]
    
    for filename in files:
        try:
            # Read the file content
            with open(filename, 'r') as file:
                lines = file.readlines()
            
            # Check if file is empty
            if not lines:
                print(f"Warning: {filename} is empty.")
                continue
            
            # Process each line to replace label 1 with 0
            new_lines = []
            for line in lines:
                parts = line.split()
                if parts[0] != '0':
                    parts[0] = '0'
                new_lines.append(' '.join(parts) + '\n')
            
            # Write the modified content back to the file
            with open(filename, 'w') as file:
                file.writelines(new_lines)
                
        except Exception as e:
            print(f"An error occurred while processing {filename}: {e}")

# Run the function to process the files
process_files_in_directory()
