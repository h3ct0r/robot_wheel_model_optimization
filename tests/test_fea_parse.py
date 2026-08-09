"""Reading CalculiX text output.

The fixtures here are trimmed from real CalculiX 2.23 output. That matters: the exact
block layout — in particular that **element** output carries both an element number and an
integration-point number before the values, while nodal output carries only a node number —
is not something to guess. Getting it wrong yields the integration-point index where sxx
should be, which reads as a small, entirely plausible stress.
"""

from __future__ import annotations

import unittest

import numpy as np

from wheelopt.fea.parse import collect, parse_dat, parse_sta

# Real CalculiX 2.23 output, trimmed.
DAT = """
                        S T E P       1


                                INCREMENT     1


 displacements (vx,vy,vz) for set NREF and time  0.5000000E+00

     85546  0.000000E+00 -2.000000E-03  0.000000E+00

 forces (fx,fy,fz) for set NREF and time  0.5000000E+00

     85546  1.234000E-14  4.210000E+01 -5.600000E-15

 stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set ESPOKES and time  0.5000000E+00

         1   1  8.989320E+02  3.200000E+01 -1.100000E+01  4.000000E+00  1.000000E+00  2.000000E+00
         1   2  9.100000E+02  3.300000E+01 -1.200000E+01  4.100000E+00  1.100000E+00  2.100000E+00
         2   1  1.010000E+03  3.400000E+01 -1.300000E+01  4.200000E+00  1.200000E+00  2.200000E+00

 displacements (vx,vy,vz) for set NREF and time  0.1000000E+01

     85546  0.000000E+00 -4.000000E-03  0.000000E+00

 forces (fx,fy,fz) for set NREF and time  0.1000000E+01

     85546  2.000000E-14  9.870000E+01 -1.100000E-14
"""

STA = """
  step inc att iter tot time  step time  inc time
     1   1   1    4  0.020000  0.020000  0.020000
     1   2   1    3  0.045000  0.045000  0.025000
     1   3   2    9  0.057500  0.057500  0.012500
     1   4   1    5  0.082500  0.082500  0.025000
"""


class TestParseDat(unittest.TestCase):
    def setUp(self):
        self.blocks = parse_dat(DAT)

    def test_finds_every_block(self):
        self.assertEqual(len(self.blocks), 5)

    def test_classifies_quantities(self):
        kinds = {b.quantity for b in self.blocks}
        self.assertEqual(kinds, {"displacement", "force", "stress"})

    def test_reads_times(self):
        times = sorted({b.time for b in self.blocks})
        self.assertEqual(times, [0.5, 1.0])

    def test_nodal_block_has_one_id_column(self):
        d = collect(self.blocks, "displacement")[1.0]
        self.assertEqual(d.ids.tolist(), [85546])
        self.assertEqual(d.values.shape, (1, 3))
        self.assertAlmostEqual(float(d.values[0, 1]), -4.0e-3)

    def test_element_block_has_two_id_columns(self):
        """The regression that matters: sxx must not be the integration point index."""
        s = collect(self.blocks, "stress")[0.5]
        self.assertEqual(s.ids.tolist(), [1, 1, 2])
        self.assertEqual(s.sub_ids.tolist(), [1, 2, 1])
        self.assertEqual(s.values.shape, (3, 6))
        self.assertAlmostEqual(float(s.values[0, 0]), 898.932)

    def test_component_names_come_from_the_header(self):
        s = collect(self.blocks, "stress")[0.5]
        self.assertEqual(s.components, ("sxx", "syy", "szz", "sxy", "sxz", "syz"))
        d = collect(self.blocks, "displacement")[0.5]
        self.assertEqual(d.components, ("vx", "vy", "vz"))

    def test_force_is_read(self):
        f = collect(self.blocks, "force")[1.0]
        self.assertAlmostEqual(float(f.values[0, 1]), 98.7)


class TestRealBlockVariants(unittest.TestCase):
    """The four leading-column conventions CalculiX 2.23 actually emits in one file.

    All four appeared in a single wheel run; each is captured verbatim here because each
    one broke the parser in a different, silent way.
    """

    def test_totals_only_force_has_no_id_column(self):
        """`*NODE PRINT, TOTALS=ONLY` prints a bare sum with no node number at all.

        Assuming one id column consumes fx as the id and drops fz off the end.
        """
        text = (
            " total force (fx,fy,fz) for set NREF and time  0.5000000E+00\n"
            "\n"
            "        3.559716E-03  1.872841E+00 -1.335474E-03\n"
        )
        block = parse_dat(text)[0]
        self.assertEqual(block.quantity, "total_force")
        self.assertEqual(block.values.shape, (1, 3))
        self.assertAlmostEqual(float(block.values[0, 1]), 1.872841)

    def test_contact_stress_names_its_id_column(self):
        """`(slave node,press,tang1,tang2)` — the id is named, unlike nodal output."""
        text = (
            " contact stress (slave node,press,tang1,tang2) for all contact elements"
            " and time 0.5000000E+00\n"
            "      5018  1.762863E+05 -1.137097E+03  4.555442E+03\n"
        )
        block = parse_dat(text)[0]
        self.assertEqual(block.quantity, "contact_stress")
        self.assertEqual(block.ids.tolist(), [5018])
        self.assertEqual(block.values.shape, (1, 3))
        self.assertAlmostEqual(float(block.values[0, 0]), 1.762863e5)

    def test_relative_contact_displacement_is_not_nodal_displacement(self):
        """The regression that emptied the load curve.

        Matching 'displacement' before 'contact' classified this block as nodal
        displacement, where it then overwrote the reference-node history at the same time
        and `build_load_curve` found nothing to work with.
        """
        text = (
            " relative contact displacement (slave node,normal,tang1,tang2) for all"
            " contact elements and time 0.5000000E+00\n"
            "      5018  1.0E-06  2.0E-07  3.0E-07\n"
            " displacements (vx,vy,vz) for set NREF and time  0.5000000E+00\n"
            "     20902  0.000000E+00  2.000000E-03  0.000000E+00\n"
        )
        blocks = parse_dat(text)
        self.assertEqual(
            {b.quantity for b in blocks}, {"contact_displacement", "displacement"}
        )
        nodal = collect(blocks, "displacement")[0.5]
        self.assertEqual(nodal.ids.tolist(), [20902])

    def test_contact_element_count_is_classified_separately(self):
        text = " total number of contact elements for time  0.5000000E+00\n         3\n"
        blocks = parse_dat(text)
        self.assertTrue(all(b.quantity != "total_force" for b in blocks))


class TestParseDatRobustness(unittest.TestCase):
    """Invariant 4: a malformed file is a typed failure upstream, never an exception."""

    def test_empty_input(self):
        self.assertEqual(parse_dat(""), [])

    def test_whitespace_only(self):
        self.assertEqual(parse_dat("\n\n   \n"), [])

    def test_truncated_mid_block(self):
        truncated = DAT[: DAT.index("stresses") + 120]
        blocks = parse_dat(truncated)
        self.assertGreater(len(blocks), 0)

    def test_garbage_does_not_raise(self):
        for junk in ("\x00\x01\x02", "not a dat file", "*** ERROR ***", "1 2 3\n4 5 6"):
            with self.subTest(junk=junk[:12]):
                parse_dat(junk)

    def test_header_without_a_time_is_kept_with_nan(self):
        blocks = parse_dat("stresses (elem, integ.pnt.,sxx) for set E\n 1 1 5.0\n")
        self.assertEqual(len(blocks), 1)
        self.assertTrue(np.isnan(blocks[0].time))

    def test_collect_drops_blocks_without_a_time(self):
        blocks = parse_dat("stresses (elem, integ.pnt.,sxx) for set E\n 1 1 5.0\n")
        self.assertEqual(collect(blocks, "stress"), {})


class TestParseSta(unittest.TestCase):
    def test_counts_increments_and_cutbacks(self):
        sta = parse_sta(STA)
        self.assertEqual(sta.n_increments, 4)
        self.assertEqual(sta.n_cutbacks, 1)  # the attempt-2 row
        self.assertAlmostEqual(sta.final_time, 0.0825)

    def test_empty_is_zeroed_not_an_error(self):
        sta = parse_sta("")
        self.assertEqual(sta.n_increments, 0)
        self.assertEqual(sta.final_time, 0.0)

    def test_header_only(self):
        self.assertEqual(parse_sta("  step inc att iter tot time\n").n_increments, 0)

    def test_garbage_does_not_raise(self):
        parse_sta("\x00 nonsense \n 1 2\n")


if __name__ == "__main__":
    unittest.main()
