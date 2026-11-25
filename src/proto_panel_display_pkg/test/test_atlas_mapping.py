from array import array
import copy
import unittest
from proto_panel_display_pkg.atlas_mapping import AtlasMap


def configuration():
    return dict(schema=1,side_width=2,side_height=1,leds_per_strip=2,num_strips=2,
                pixels=[['left',0,0],['right',1,0],None,['right',0,0]])


class AtlasMappingTests(unittest.TestCase):
    def test_side_order_rgb_conversion_and_unused_slot(self):
        mapping=AtlasMap(configuration())
        bgr=array('B',[1,2,3, 4,5,6, 7,8,9, 10,11,12])
        self.assertEqual(mapping.rgb_bytes(bgr,4,1),bytes([3,2,1,12,11,10,0,0,0,9,8,7]))
        self.assertEqual(mapping.rgb_bytes(list(bgr),4,1),mapping.rgb_bytes(bgr,4,1))

    def test_invalid_map_or_payload_is_rejected(self):
        for key,value in [('num_strips',3),('side_width',0),('pixels',[['left',2,0]]*4),
                          ('pixels',[['unknown',0,0]]*4),('pixels',[['left',True,0]]*4)]:
            config=copy.deepcopy(configuration());config[key]=value
            with self.assertRaises(ValueError):AtlasMap(config)
        mapping=AtlasMap(configuration())
        for data,width,height,flip in [(b'bad',4,1,False),(bytes(12),2,2,False),(bytes(12),4,1,True)]:
            with self.assertRaises(ValueError):mapping.rgb_bytes(data,width,height,flip)


if __name__=='__main__':unittest.main()
