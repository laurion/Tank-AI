def select_view_module(name):
    global selectedvisual
    if name == 'pygame':
        import visual
        selectedvisual = visual

    elif name == 'none':
        import novisual
        selectedvisual = novisual

def get_view_module():
    global selectedvisual
    return selectedvisual
