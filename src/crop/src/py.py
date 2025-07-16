import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

# 샘플 차선 데이터 (라이다에서 추출했다고 가정)
np.random.seed(42)
x = np.linspace(-10, 10, 100)
y = 0.5 * x**2 + 2*x + np.random.normal(0, 2, size=x.shape)  # 실제 차선은 곡선
outliers_x = np.random.uniform(-10, 10, 20)  # 이상치 추가
outliers_y = np.random.uniform(-10, 50, 20)

# 이상치 추가한 데이터
X = np.concatenate([x, outliers_x]).reshape(-1, 1)
Y = np.concatenate([y, outliers_y])

# RANSAC 모델 (Polynomial Regression)
ransac = RANSACRegressor()
ransac.fit(X, Y)

# Inlier와 Outlier 구분
inlier_mask = ransac.inlier_mask_
outlier_mask = ~inlier_mask

# 다항식 피팅 (Polynomial Regression)
poly_model = make_pipeline(PolynomialFeatures(degree=2), RANSACRegressor())
poly_model.fit(X[inlier_mask], Y[inlier_mask])  # RANSAC 인라이어만 사용

# 예측
x_fit = np.linspace(-10, 10, 100).reshape(-1, 1)
y_fit = poly_model.predict(x_fit)

# 결과 시각화
plt.scatter(X[inlier_mask], Y[inlier_mask], color="blue", label="Inliers")
plt.scatter(X[outlier_mask], Y[outlier_mask], color="red", label="Outliers")
plt.plot(x_fit, y_fit, color="green", linewidth=2, label="Fitted Polynomial Curve")
plt.xlabel("X (Distance)")
plt.ylabel("Y (Lane Position)")
plt.legend()
plt.title("RANSAC Lane Detection with Polynomial Fitting")
plt.show()
