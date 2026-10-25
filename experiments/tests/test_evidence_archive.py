import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from archive_plaza_evidence import verify


class ArchiveVerificationTests(unittest.TestCase):
    def setUp(self):
        parent=Path(__file__).resolve().parents[1]/'verification'
        self.temp=tempfile.TemporaryDirectory(prefix='archive_test_',dir=parent)
        self.root=Path(self.temp.name).resolve()
        assert self.root.is_relative_to(parent.resolve())
        self.addCleanup(self.temp.cleanup)
        (self.root/'sample.bin').write_bytes(b'abc')
        self.manifest={'payload_bytes':3,'payload_file_count':1,
                       'files':[{'path':'sample.bin','bytes':3,'sha256':hashlib.sha256(b'abc').hexdigest()}]}
        self.seal()

    def seal(self):
        content=json.dumps(self.manifest).encode('utf-8')
        (self.root/'MANIFEST.json').write_bytes(content)
        (self.root/'MANIFEST.sha256').write_text(hashlib.sha256(content).hexdigest()+'  MANIFEST.json\n')

    def test_accepts_intact_copy(self):
        self.assertEqual(verify(self.root)['verified_payload_files'],1)

    def test_rejects_changed_bytes_even_when_size_matches(self):
        (self.root/'sample.bin').write_bytes(b'xyz')
        with self.assertRaises(ValueError):verify(self.root)

    def test_rejects_unlisted_file(self):
        (self.root/'extra.bin').write_bytes(b'x')
        with self.assertRaises(ValueError):verify(self.root)

    def test_rejects_changed_manifest(self):
        with (self.root/'MANIFEST.json').open('a') as stream:stream.write(' ')
        with self.assertRaises(ValueError):verify(self.root)

    def test_rejects_path_outside_archive(self):
        self.manifest['files'][0]['path']='../outside.bin';self.seal()
        with self.assertRaises(ValueError):verify(self.root)


if __name__=='__main__':unittest.main()
