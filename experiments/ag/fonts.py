from PIL import ImageFont
import os

def font(size, bold=False):
    names = [os.environ.get('AIRSIMSGA_FONT'), 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf', 'arialbd.ttf' if bold else 'arial.ttf']
    for name in names:
        if not name:continue
        try:return ImageFont.truetype(name,size)
        except OSError:pass
    raise RuntimeError('Install DejaVu Sans or set AIRSIMSGA_FONT to a .ttf font')
