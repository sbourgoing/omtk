from omtk.core.classNomenclature import Nomenclature


class NomenclatureSnakeCase(Nomenclature):
    separator = '_'

    @classmethod
    def split(cls, val):
        val.split(cls.separator)

    @classmethod
    def join(cls, tokens):
        return cls.separator.join(tokens)


def register_plugin():
    return Nomenclature
