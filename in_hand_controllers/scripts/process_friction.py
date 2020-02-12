    pic_data = pickle.load( open( f_name_, "rb" ) )
    #print pic_data
    obj_keys = pic_data.keys()
    obj_keys.sort()
    print obj_keys

    # gt values:
    gt  = {"mustard_l":0.59,"mustard":0.64,"ss_l":0.45,"ss":0.44,
           "lego":0.68,"cheezit":0.61, "sugar":0.47,"wood":0.62,
           "peg":0.62,"3d-print":0.59}
    #print gt.keys()

    filter_keys = ["3d_box_1","cracker_3","lego","mustard_1","mustard_label","ss_label_2",
                   "ss_plastic_2","sugar_box","wood"]
    rename_k = ["3d-print","cheezit","lego","mustard","mustard_l","ss_l","ss",
                "sugar","wood"]

  
    new_d = {}
    for j in range(len(filter_keys)):
        data = pic_data[filter_keys[j]]
        mu = []
        for i in range(len(data)):
            #mu.append(data[i]['mu'])
            mu.append(gt[rename_k[j]] - data[i]['mu'])
            #mu.append(np.abs((gt[rename_k[j]] - data[i]['mu'])))
        new_d[rename_k[j]] = mu
