import unittest
from kineticguard.compliance.machinery_audit import generate_machinery_audit


class TestMachineryCompliance(unittest.TestCase):

    def test_generate_machinery_audit(self):
        cert = generate_machinery_audit(
            robot_id="franka_emika_01",
            total_cycles=100000,
            interventions=240,
            reflex_stops=1,
            zero_breach=True,
            p99_us=280.0
        )

        self.assertEqual(cert.robot_id, "franka_emika_01")
        self.assertTrue(cert.zero_boundary_breach)
        self.assertIn("ISO 13849-1", cert.standards_satisfied[0])

        digest = cert.compute_sha256_digest()
        self.assertTrue(digest.startswith("sha256:"))

        md = cert.generate_markdown()
        self.assertIn("CERTIFIED CONFORMANT", md)
        self.assertIn(digest, md)


if __name__ == "__main__":
    unittest.main()
