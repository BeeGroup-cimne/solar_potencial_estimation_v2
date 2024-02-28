"""
List of functions of general interest
### Public functions:
#### create_output_folder: creates a folder for the given directory
#### delete_folder: deletes directory and its contents.
#### merge_txt: Given a list of txt files, merges them and saves them in a destination file. Returns the export directory.
"""
import os # Interface with system directories
import shutil # High-level file operations

def create_output_folder(directory, deleteFolder=False):
    """
    Creates a folder for the given directory, only if it didn't exist before.
    If the directory already existed, it does nothing

    #### Inputs:
    - directory: folder (complete path) to create
    - deleteFolder: to delete or not the folder in case it already existed

    #### Outputs:
    - None 

    ### Exports:
    - The specified folder is created
    """

    if not(os.path.isdir(directory)):
       os.makedirs(directory)
    else:
        if(deleteFolder):
            shutil.rmtree(directory) # Deletes directory and its contents
            os.makedirs(directory)  # Creates new directory    

def delete_folder(directory):
    """
    Deletes the given directory

    #### Inputs:
    - directory: folder (complete path) to delete

    #### Outputs:
    - None
    
    ### Exports:
    - None. The specified folder is deleted
    """
    shutil.rmtree(directory) # Deletes directory and its contents


def merge_txt(filenames, destination):
    """
    Given a list of txt files, merges them and saves them in a destination file.
    Returns the export directory.
    If the list contains only one file, it does nothing.

    #### Inputs:
    - filenames: array of txt files to merge
    - destination: file to store the merged results

    #### Outputs:
    - it returns the export directory. If the list contains only one file, it returns that same filename

    ### Exports:
    - if merging is succesful, .csv containing the points of all the files in filenames
    """
    # We will not do anything if the list contains one file (or less)
    if(len(filenames) <= 1):
        return filenames[0]
    
    else:
        # Open outputFile in write mode
        with open(destination, 'w') as outfile:
            # outfile.write("x_coord y_coord z_coord\n") # Do not uncomment with current configuration
                # Iterate through list
            for names in filenames:
                    # Open each file in read mode
                with open(names) as infile:
                        # read the data from each readfile and writes it in outputfile
                    outfile.write(infile.read())
                infile.close()
                # Add '\n' to enter data of the following file from next line
                outfile.write("\n")
                
        outfile.close()

        return destination

if __name__ == "__main__":
    print("General functions Module")