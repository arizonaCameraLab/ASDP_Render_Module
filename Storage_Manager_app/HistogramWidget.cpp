#include "HistogramWidget.h"
#include <QPainter>
#include <algorithm>

HistogramWidget::HistogramWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(256, 128); // Adjust as needed
}

void HistogramWidget::setHistogram(const std::vector<int>& hist)
{
    m_histogram = hist;
    update();
}

void HistogramWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.fillRect(rect(), Qt::white);

    if (m_histogram.empty())
        return;

    int bins = m_histogram.size();
    int w = width();
    int h = height();
    int maxVal = *std::max_element(m_histogram.begin(), m_histogram.end());
    if (maxVal == 0) maxVal = 1;

    double binWidth = static_cast<double>(w) / bins;
    for (int i = 0; i < bins; ++i) {
      int binHeight = int(double(m_histogram[i]) / maxVal * (h - 10));
      QRectF rect(i * binWidth, h - binHeight, binWidth, binHeight);
      painter.setBrush(Qt::black);
      painter.setPen(Qt::NoPen);
      painter.drawRect(rect);
    }
}

