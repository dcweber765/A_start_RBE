


def optimize_path(path):
    """datadata
        remove redundant points in hte path
        :param path: list of tuples
        :return: reduced list of tuples
    """

    #Check X straight line
    #Check Y straight line
    #Check Diga Line

    #check slope
    #check rise to run between 3 adjacent points

    #currentPath = Path
    #OptPath = []
    #grab Oth i-2
    #grab 1st i-1
    #iterate through list starting at 2 (i)
        #s1 = (i-2[1] - i-1[1]) / (i-2[0] - i-1[0])
        #s2 = (i-1[1] - i[1]) / (i-1[0] - i[0])
        #if s1 == s2
            #del currentPath[i-1]

    currentPath = path
    print currentPath
    ZeroTh = currentPath[0]
    First =  currentPath[1]
    i = 2

    for counter, value in enumerate(currentPath[2:],2):
        if counter < len(currentPath):
            print(counter, value)
            print currentPath[counter - 2]
            print '\n'
            print currentPath[counter-1]
            print '\n'
            print currentPath[counter]
            slope1 = (currentPath[counter-2][1] - currentPath[counter-1][1]) / (currentPath[counter-2][0] - currentPath[counter-1][0])
            slope2 = (currentPath[counter-1][1] - currentPath[counter][1]) / (currentPath[counter-1][0] - currentPath[counter][0])

            print slope1
            print slope2
            if slope1 == slope2:
                del currentPath[counter-1]

            print currentPath

    return currentPath


if __name__ == "__main__":
    path = [(2, 2), (3, 3), (3, 4), (4, 5), (4, 6), (4, 7), (4, 8), (4, 9), (4, 10), (4, 11), (4, 12), (4, 13), (4, 14), (4, 15), (4, 16), (4, 17), (4, 18), (4, 19), (5, 20), (6, 20), (7, 20), (8, 20), (9, 20), (10, 20), (11, 20), (12, 21), (13, 22), (14, 23), (15, 24), (16, 25), (17, 26), (18, 27), (19, 28), (20, 29), (21, 30), (22, 31), (23, 32), (24, 32), (25, 32), (26, 32), (27, 32), (28, 32), (29, 32), (30, 32), (31, 32), (32, 31), (32, 30), (32, 29), (31, 28), (30, 27), (29, 26), (28, 25), (27, 24), (26, 23), (26, 22), (26, 21), (26, 20), (26, 19), (26, 18), (26, 17), (26, 16), (26, 15), (26, 14), (26, 13), (26, 12), (26, 11), (25, 10), (24, 9), (23, 8), (22, 7), (21, 6), (20, 5), (19, 4), (18, 4), (17, 4), (16, 4), (15, 4), (14, 4), (13, 4), (12, 4), (11, 4), (10, 5), (9, 6)]
    print optimize_path(path)