from pgmpy.inference import VariableElimination
from collections import defaultdict
import pandas as pd

def bn_predict(model,data,headers):
    missing_variables = set(model.nodes()) - set(data.columns)
    model_inference = VariableElimination(model)

    preds = []
    for _, data_point in data.iterrows():
        pred = model_inference.map_query(variables=missing_variables,evidence=data_point.to_dict(),show_progress=False)
        row = []
        for h in headers:
            row.append(pred[h])
        preds.append(row)
    return pd.DataFrame(preds, index=data.index, columns=headers)