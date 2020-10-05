import pickle

with open('res_dict.txt', 'rb') as fin:
    with open('res_dict_text.txt', 'w') as fout:
        obj = pickle.load(fin)
        for key in obj.keys():
            string = str(key) + ', ' + str(obj[key][0]) + ', ' + str(obj[key][1])
            fout.write(string + '\n')
