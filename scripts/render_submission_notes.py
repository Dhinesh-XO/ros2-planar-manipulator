#!/usr/bin/env python3
"""Render the two short Markdown answers as printable assessment notes.

Uses reportlab only for documentation, not at controller runtime. This deliberately
small renderer accepts a title plus plain paragraphs, as used by these two notes.
"""

import argparse
from html import escape
from pathlib import Path
import re

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.platypus import Paragraph, SimpleDocTemplate

ROOT = Path(__file__).resolve().parents[1]


def markup(text):
    return re.sub(r'`([^`]+)`', r'<font name="Courier">\1</font>', escape(text))


def render(source, destination):
    blocks = source.read_text().strip().split('\n\n')
    title = blocks.pop(0).removeprefix('# ')
    styles = {
        'title': ParagraphStyle('Title', fontName='Helvetica-Bold', fontSize=16,
                                leading=20, spaceAfter=10, textColor=colors.HexColor('#17394b')),
        'body': ParagraphStyle('Body', fontName='Helvetica', fontSize=10.5,
                               leading=14, spaceAfter=8),
    }
    story = [Paragraph(markup(title), styles['title'])]
    for block in blocks:
        story.append(Paragraph(markup(' '.join(block.splitlines())), styles['body']))

    def footer(canvas, document):
        canvas.saveState()
        canvas.setFont('Helvetica', 8)
        canvas.setFillColor(colors.HexColor('#666666'))
        canvas.drawString(18*mm, 12*mm, 'Kineshia Robotics assessment | ROS 2 Planar Manipulator')
        canvas.drawRightString(A4[0]-18*mm, 12*mm, str(document.page))
        canvas.restoreState()

    document = SimpleDocTemplate(str(destination), pagesize=A4,
                                 leftMargin=18*mm, rightMargin=18*mm,
                                 topMargin=17*mm, bottomMargin=20*mm,
                                 title=title, author='ROS 2 Planar Manipulator')
    document.build(story, onFirstPage=footer, onLaterPages=footer)
    print(destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/submission-notes')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name in ('DESIGN_NOTE', 'HARDWARE_TRANSITION'):
        render(ROOT / 'docs' / f'{name}.md', args.output / f'{name}.pdf')


if __name__ == '__main__':
    main()
