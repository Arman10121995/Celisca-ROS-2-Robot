"""License identification from text, never from names (R9.1)."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('external_source_licenses',
    Path(__file__).resolve().parents[3] / 'scripts' / 'external_source_licenses.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_identify_common_licenses():
    bsd3 = ('Redistribution and use in source and binary forms, with or without modification, are permitted ... '
            'Neither the name of the copyright holder nor the names of its contributors may be used to endorse')
    assert module.identify(bsd3) == 'BSD-3-Clause'
    assert module.identify('Redistribution and use in source and binary forms, with or without modification') == 'BSD-2-Clause'
    assert module.identify('Apache License\nVersion 2.0, January 2004') == 'Apache-2.0'
    assert module.identify('Permission is hereby granted, free of charge, to any person') == 'MIT'
    assert module.identify('GNU LESSER GENERAL PUBLIC LICENSE Version 3, 29 June 2007') == 'LGPL-3.0'
    assert module.identify('Some custom terms') is None


def test_tree_without_license_file_stays_unresolved(tmp_path):
    (tmp_path / 'package.xml').write_text('<package><license>MIT</license></package>')
    row = module.scan_tree(tmp_path)
    assert row['status'] == 'unresolved' and row['package_xml'] == ['MIT']
    (tmp_path / 'LICENSE').write_text('Permission is hereby granted, free of charge, to any person')
    assert module.scan_tree(tmp_path)['status'] == 'resolved'
