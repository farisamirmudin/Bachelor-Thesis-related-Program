from plotlinebestfit import plotlinebestfit
def tabletotex():
    # pathtofile = input('Enter full path to file: ')
    pathtofile = 'C:/Users/faris/Documents/stuff/literature/tabletest.txt'
    # caption = input("Caption: ")
    # label = input("Label: ")

    with open(pathtofile, 'r') as f:
        datas = f.readlines()

    datalist = []
    for data in datas:
        datalist.append([s for s in data.strip().split(' ')])

    texstr = '\\begin{table}[h]\n'
    colnr = len(datalist[0])
    colstr = '|c'*colnr + '|'
    texstr = texstr + '\\begin{tabular}{' + colstr + '}\n\\hline\n'
    error = []
    distance = []
    for row in datalist:
        colstr = ''
        for col in row:
            colstr += col
            if col == row[0]:
                distance.append(col)
            if col == row[2]:
                error.append(col)
            if col == row[-1]:
                colstr += '\\\\\n\\hline\n'
            else:
                colstr += '&'
        texstr += colstr

    texstr += '\\end{tabular}\n\\caption{}\n\\label{table:}\n\\end{table}'
    return distance, error, texstr

def main():
    distance, error, texstr = tabletotex()
    print(texstr)
    plotlinebestfit(distance, error)

if __name__ == '__main__':
    main()
