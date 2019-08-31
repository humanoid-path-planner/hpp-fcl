#!/usr/bin/env python3
import pdb

checks =  [ "AB", "AC", "AD" ] \
        + [ "ABC", "ACD", "ADB" ] \
        + [ "ABC.cross({})".format(n) for n in [ "AB", "AC" ] ] \
        + [ "ACD.cross({})".format(n) for n in [ "AC", "AD" ] ] \
        + [ "ADB.cross({})".format(n) for n in [ "AD", "AB" ] ]

tests = list(range(len(checks)))

weights = [ 1, ] * 3 + [ 2, ] * 3 + [ 3, ] * 6

regions = [ "ABC", "ACD", "ADB", "AB", "AC", "AD", "A", "Inside", ]
cases = list(range(len(regions)))

definitions = [
        [  1,  4,- 5 ],
        [  2,  6,- 7 ],
        [  3,  8,- 9 ],
        [- 4,  9, 10 ],
        [- 6,  5, 11 ],
        [- 8,  7, 12 ],
        [-10,-11,-12 ],
        [- 1,- 2,- 3 ],
        ]
conditions = [
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [ [10, 11, 12], ],
        ]
rejections = [
        [],
        [],
        [],
        [ [- 1,- 3], ],
        [ [- 2,- 1], ],
        [ [- 3,- 2], ],
        [ [  4,- 5], [  6,- 7], [  8,- 9], ],
        [],
        ]

def apply_test_values (cases, test_values):
    def canSatisfy (values, indices):
        for k in indices:
            if k > 0 and values[ k-1] == False: return False
            if k < 0 and values[-k-1] == True : return False
        return True
    def satisfies (values, indices):
        for k in indices:
            if k > 0 and values[ k-1] != True : return False
            if k < 0 and values[-k-1] != False: return False
        return True

    # Check whether all cases are 
    left_cases = []
    for case in cases:
        defi = definitions[case]
        conds = conditions[case]
        rejs = rejections[case]
        if satisfies (test_values, defi):
            # A definition is True, stop recursion
            return [ case ]
        if not canSatisfy (test_values, defi):
            continue
        for cond in conds:
            if satisfies (test_values, cond):
                # A condition is True, stop recursion
                return [ case ]
        append = True
        for rej in rejs:
            if satisfies (test_values, rej):
                # A rejection is True, discard this case
                append = False
                break
        if append: left_cases.append (case)
    return left_cases

def max_number_of_tests (current_tests, cases, test_values = [None,]*len(tests), prevBestScore = float('inf'), prevScore = 0):
    for test in current_tests:
        assert test_values[test] == None, "Test " +str(test)+ " already performed"

    left_cases = apply_test_values (cases, test_values)

    if len(left_cases) == 1:
        return prevScore, { 'case': left_cases[0], }
    elif len(left_cases) == 0:
        return prevScore, { }

    assert len(current_tests) > 0, "No more test but " + str(left_cases) + " remains"

    remaining_tests = current_tests[1:]
    currentBestScore = prevBestScore
    bestScore = float('inf')
    bestOrder = [None, None]
    for i, test in enumerate(current_tests):
        assert bestScore >= currentBestScore

        if i > 0:
            remaining_tests[i-1] = current_tests[i-1]

        currentScore = prevScore + len(left_cases) * weights[test]
        if currentScore > currentBestScore: # Cannot do better -> stop
            continue

        test_values[test] = True
        score_if_t, order_if_t = max_number_of_tests (remaining_tests, left_cases, test_values, currentBestScore, currentScore)
        if score_if_t >= currentBestScore: # True didn't do better -> stop
            test_values[test] = None
            continue

        test_values[test] = False
        score_if_f, order_if_f = max_number_of_tests (remaining_tests, left_cases, test_values, currentBestScore, currentScore)
        test_values[test] = None

        currentScore = max(score_if_t, score_if_f)
        if currentScore < bestScore:
            if currentScore < currentBestScore:
                bestScore = currentScore
                bestOrder = { 'test': test, 'true': order_if_t, 'false': order_if_f }
                #pdb.set_trace()
                currentBestScore = currentScore
                if len(tests) == len(current_tests):
                    print ("New best score: {}".format(currentBestScore))

    return bestScore, bestOrder

def printOrder (order, indent = "", start=True):
    if start:
        print (indent+"// The code of this function was generated using doc/gjk.py")
        print (indent+"const int a = 3, b = 2, c = 1, d = 0;")
        for l in "abcd":
            print (indent+"const Vec3f& {} (current.vertex[{}]->w);".format(l.upper(),l))
        for l in "BCD":
            print (indent+"const Vec3f A{0} ({0}-A);".format(l))
            print (indent+"const FCL_REAL A{0}_dot_AO = A{0}.dot(-A);".format(l))
        for l1,l2 in zip("BCD","CDB"):
            print (indent+"const Vec3f A{0}{1} (A{0}.cross(A{1}));".format(l1,l2))
            print (indent+"const FCL_REAL A{0}{1}_dot_AO = A{0}{1}.dot(-A);".format(l1,l2))
        print (indent+"Vec3f cross;")
        print()
        print(       "#define REGION_INSIDE()               "+indent+"\\")
        print(indent+"  ray.setZero();                      \\")
        print(indent+"  next.vertex[0] = current.vertex[d]; \\")
        print(indent+"  next.vertex[1] = current.vertex[c]; \\")
        print(indent+"  next.vertex[2] = current.vertex[b]; \\")
        print(indent+"  next.vertex[3] = current.vertex[a]; \\")
        print(indent+"  next.rank=4;                        \\")
        print(indent+"  return 0;")
        print()

    if 'case' in order:
        region = regions[order['case']]
        print (indent + "// Region " + region)
        toFree = ['a', 'b', 'c', 'd']
        if region == "Inside":
            print (indent + "REGION_INSIDE()")
            toFree = []
        elif region == 'A':
            print (indent + "originToPoint (current, a, A, next, ray);")
            toFree.remove('a')
        elif len(region)==2:
            a = region[0]
            b = region[1]
            print (indent + "originToSegment (current, {}, {}, {}, {}, {}, {}_dot_AO, next, ray);".format(
                a.lower(), b.lower(), a, b, region, region))
            toFree.remove(a.lower())
            toFree.remove(b.lower())
        elif len(region)==3:
            a = region[0]
            b = region[1]
            c = region[2]
            print (indent + "originToTriangle (current, {}, {}, {}, {}, {}_dot_AO next, ray);".format(
                a.lower(), b.lower(), c.lower(), region, region))
            toFree.remove(a.lower())
            toFree.remove(b.lower())
            toFree.remove(c.lower())
        else:
            assert False, "Unknown region " + region
        for pt in toFree:
            print (indent + "free_v[nfree++] = current.vertex[{}];".format(pt))
    else:
        assert "test" in order and 'true' in order and 'false' in order
        check = checks[order['test']]
        if 'cross' in check:
            print (indent + "cross.noalias() = {};".format(checks[order['test']]))
            print (indent + "if (cross.dot(-A) >= 0) {")
        elif len(check) == 2 or len(check) == 3:
            print (indent + "if ({}_dot_AO >= 0) {{".format(checks[order['test']]))
        else:
            assert False, "Unknown check " + check
            #print (indent + "if ({}.dot(-A) >= 0) {{".format(checks[order['test']]))
        printOrder (order['true'], indent=indent+"  ", start=False)
        print (indent + "} else {")
        printOrder (order['false'], indent=indent+"  ", start=False)
        print (indent + "}")

    if start:
        print()
        print("#undef REGION_INSIDE")
        print(indent+"return ray.squaredNorm();")

score, order = max_number_of_tests (tests, cases)

print(score)
printOrder(order, indent="  ")

# TODO add weights such that:
# - it is preferred to have all the use of one check in one branch.
#   idea: ponderate by the number of remaining tests.
