# this your custom configuration module
from PyQt4.Qt import *
from sloth import items
from sloth.items import BaseItem
from sloth.conf.default_config import LABELS
# class MyRectItem(RectItem):
#     def __init__(self, index, data):
#         # Call the base class constructor.  This will make the label
#         # data available in self.data
#         BaseItem.__init__(self, index, data)

#         # Create a new rect item and add it as child item.
#         # This defines what will be displayed for this label, since the
#         # BaseItem base class itself does not display anything.
#         x, y, x_width, y_height = map(float, (self.data['x'],     self.data['y'],
#                                           self.data['width'], self.data['height']))
#         self.rect_ = QGraphicsRectItem(x, y, x_width, y_height, self)


RedRectItem = items.RectItem()
RedRectItem.setColor(Qt.red)
GreenRectItem = items.RectItem()
GreenRectItem.setColor(Qt.green)
YellowRectItem = items.RectItem()
YellowRectItem.setColor(Qt.yellow)


# ITEMS = {
#     "Green" : GreenRectItem,
#     "Yellow" : GreenRectItem,
#     "Red" : GreenRectItem,
# }

MYLABELS = (
	{
        'attributes': {
            'class':      'Green',
         },
        'inserter': 'sloth.items.RectItemInserter',
        'item':     'sloth.items.RectItem',
        'hotkey':   'r',
        'text':     'Green',
    },
    {
        'attributes': {
            'class':      'Yellow',
         },
        'inserter': 'sloth.items.RectItemInserter',
        'item':     'sloth.items.RectItem',
        'hotkey':   'r',
        'text':     'Yellow',
    },
    {
        'attributes': {
            'class':      'Red',
         },
        'inserter': 'sloth.items.RectItemInserter',
        'item':     'sloth.items.RectItem',
        'hotkey':   'r',
        'text':     'Red',
    },
)
LABELS += MYLABELS