name = 'omtk'

version = '0.0.17'

requires = ['libSerialization']

def commands():
    env.PYTHONPATH.append('{root}/scripts')
    env.MAYA_MODULE_PATH.append('{root}')
