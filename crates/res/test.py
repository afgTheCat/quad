import res
import scipy
from sklearn.preprocessing import OneHotEncoder

data = scipy.io.loadmat("../../data/JpVow.mat")

Xtr, Ytr, Xte, Yte = data["X"], data["Y"], data["Xte"], data["Yte"]

onehot_encoder = OneHotEncoder(sparse_output=False)
Ytr = onehot_encoder.fit_transform(Ytr)
Yte = onehot_encoder.transform(Yte)

res = res.ClassicESNModel(500)

