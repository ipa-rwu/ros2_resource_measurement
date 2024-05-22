import csv
 
def convert_to_float(value):
    try:
        return float(value)
    except ValueError:
        return None
 
with open('/home/celltower1/model_ws/src/log_5_no.csv', 'r') as file:
    # Create a CSV reader object
    csv_reader = csv.reader(file)
 
    # Skip the header if it exists
    next(csv_reader, None)
    values = []
    mx = 0.0

    for row in csv_reader:
        print(row[1])
        converted_value = convert_to_float(row[1])

        if converted_value is not None:
            values.append(converted_value)
            if converted_value > mx:
                mx = converted_value
        # Iterate through each value in the row
        # for value in row[1]:
        #     converted_value = convert_to_float(value)
        #     # print(converted_value)
 
        #     # Check if the conversion was successful (not None)
        #     if converted_value is not None:
        #         values.append(converted_value)
 
        # Check if there are numeric values in the row
    if values:
        print(values)
        # Calculate the row average
        row_average = sum(values) / len(values)

        # Print the result
        print(f"mx: {mx}, Average: {row_average}")