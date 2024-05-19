import os

lines = os.listdir('./sdf/1.11/')
# Only keep the files that end with .sdf
lines = [line for line in lines if line.endswith('.sdf')]

for line in lines:
    # Remove .sdf extension
    snake_case = line.strip().split('.')[0]
    
    # Convert from snake_case to CamelCase
    camel_case = ''.join([word.capitalize() for word in snake_case.split('_')])

    addHeader = """
    /// \\brief Get the schema file name accessor
    public: static inline std::string_view SchemaFile();
    """

    addImpl = f"""
/////////////////////////////////////////////////
inline std::string_view {camel_case}::SchemaFile() 
{{
    static const char kSchemaFile[] = "{line}";
    return kSchemaFile;
}}\n\n"""
    
    # Debug print statements
    # print(addHeader)
    # print(addImpl)
    # print(addTest)
    # print("\n")

    # Edit './include/sdf/{camel_case}.hh'
    # Find the line with 'class SDFFORMAT_VISIBLE {camel_case}`
    try:
        f = open('./include/sdf/' + camel_case + '.hh', 'r')
        lines = f.readlines()
        f.close()
        line_number = 0
        for i, line in enumerate(lines):
            if line == "  class SDFORMAT_VISIBLE " + camel_case + '\n':
                line_number = i+3 # After the class line there is `\n{\n` and then the constructor
                break
        if line_number == 0:
            print("Error: Could not find class declaration in " + camel_case + ".hh")
            exit(1)

        try:
            with open('./include/sdf/' + camel_case + '.hh', 'w') as file:
                for i, line in enumerate(lines):
                    file.write(line)
                    if i == line_number:
                        file.write(addHeader)
        except:
            print("Unexpected error while writing to: " + camel_case + ".hh.")
    except:
        print("Error while writing to: " + camel_case + ".hh." + " Check if file exists.")

    # Edit './src/{camel_case}.cc' if it exists
    # Add implementation to end of document
    if os.path.exists('./src/' + camel_case + '.cc'):
        with open('./src/' + camel_case + '.cc', 'a') as file:
            file.write(addImpl)
    else:
        print("Error: Could not find " + camel_case + ".cc")

    print("Changes written to source successfully")
