#pragma once
#include <QWidget>
#include <vector>

class HistogramWidget : public QWidget
{
    Q_OBJECT
public:
    explicit HistogramWidget(QWidget* parent = nullptr);

    // Set the histogram data (256 bins for 8-bit, 65536 for 16-bit)
    void setHistogram(const std::vector<int>& hist);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    std::vector<int> m_histogram;
};

