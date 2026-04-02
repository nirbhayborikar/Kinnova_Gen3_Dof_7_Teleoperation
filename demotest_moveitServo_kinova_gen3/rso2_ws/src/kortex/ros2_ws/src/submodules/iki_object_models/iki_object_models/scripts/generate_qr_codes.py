#!/usr/bin/env python

from reportlab.pdfgen import canvas
from reportlab.graphics.shapes import Drawing, Rect
from reportlab.graphics.barcode.qr import QrCodeWidget
from reportlab.graphics import renderPDF

import os
import rospkg
rospack = rospkg.RosPack()

path = rospack.get_path('ed_object_models') + "/models"

# Create page
p = canvas.Canvas(path + "/qr_codes.pdf")

for model_name in os.listdir(path):

    # Only visible dirs
    if model_name[0] == ".":
        continue

    # Generate qr code
    qrw = QrCodeWidget(model_name)
    b = qrw.getBounds()
    w=b[2]-b[0]
    h=b[3]-b[1]

    d = Drawing(1,1,transform=[5,0,0,5,0,0])
    d.rotate(90)
    d.translate(0,-90)
    d.add(qrw)

    # Generate the rectangle
    r = Rect(1,1,1,1)

    renderPDF.draw(d, p, 0, 0)
    #renderPDF.draw(r, p, 0, 0)

    x_offset = 0;
    y_offset = 2;

    o = 30
    a = 390
    w = 30

    p.setLineWidth(w)
    p.line(x_offset+o-.5*w,y_offset+o,x_offset+o+a+.5*w,y_offset+o)
    p.line(x_offset+o,y_offset+o,x_offset+o,y_offset+o+a)
    p.line(x_offset+o+a+.5*w,y_offset+o+a,x_offset+o-.5*w,y_offset+o+a)
    p.line(x_offset+o+a,y_offset+o+a,x_offset+o+a,y_offset+o)

    p.drawString(200,450,model_name)

    p.showPage()

p.save()
