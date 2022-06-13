def totuple(a):
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a
