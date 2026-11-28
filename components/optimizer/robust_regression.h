/**
 * @file robust_regression.h
 * @brief Méthodes de régression robuste pour le système optimizer PAC
 * 
 * Implémentations de RANSAC et régression de Huber pour remplacer
 * la régression linéaire simple et résister aux outliers.
 */

#ifndef ROBUST_REGRESSION_H
#define ROBUST_REGRESSION_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

namespace esphome {
namespace optimizer {

struct RegressionResult {
    float slope;
    float intercept;
    float score;          // Métrique de qualité
    size_t inliers_count; // Nombre de points inliers (pour RANSAC)
    bool success;
};

/**
 * @brief Classe pour régression robuste avec RANSAC et Huber
 */
class RobustRegression {
public:
    /**
     * @brief Régression linéaire RANSAC (RANdom SAmple Consensus)
     * 
     * Méthode très robuste face aux outliers (jusqu'à 50% de données aberrantes).
     * Principe: tire aléatoirement des sous-ensembles, calcule le modèle,
     * et garde celui qui a le plus d'inliers.
     * 
     * @param data Vecteur de paires (x, y)
     * @param threshold Distance maximale pour qu'un point soit considéré inlier (recommandé: 1.0-2.0)
     * @param max_iterations Nombre max d'itérations (recommandé: 100-500)
     * @param min_inlier_ratio Ratio minimum d'inliers pour accepter le modèle (recommandé: 0.5-0.7)
     * @return RegressionResult avec le meilleur modèle trouvé
     */
    static RegressionResult ransac(
        const std::vector<std::pair<float, float>>& data,
        float threshold = 1.5f,
        int max_iterations = 200,
        float min_inlier_ratio = 0.6f)
    {
        RegressionResult result = {0.0f, 0.0f, 0.0f, 0, false};
        
        if (data.size() < 3) {
            ESP_LOGE("RANSAC", "Pas assez de données (besoin d'au moins 3 points)");
            return result;
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, data.size() - 1);

        size_t best_inliers_count = 0;
        float best_slope = 0.0f;
        float best_intercept = 0.0f;

        // Itérations RANSAC
        for (int iter = 0; iter < max_iterations; iter++) {
            // 1. Tirer aléatoirement 2 points distincts
            int idx1 = dis(gen);
            int idx2 = dis(gen);
            while (idx2 == idx1) {
                idx2 = dis(gen);
            }

            float x1 = data[idx1].first;
            float y1 = data[idx1].second;
            float x2 = data[idx2].first;
            float y2 = data[idx2].second;

            // 2. Calculer le modèle avec ces 2 points
            if (std::abs(x2 - x1) < 1e-6f) continue; // Éviter division par zéro

            float slope = (y2 - y1) / (x2 - x1);
            float intercept = y1 - slope * x1;

            // 3. Compter les inliers (points proches du modèle)
            size_t inliers_count = 0;
            for (const auto& point : data) {
                float predicted_y = slope * point.first + intercept;
                float error = std::abs(point.second - predicted_y);
                
                if (error <= threshold) {
                    inliers_count++;
                }
            }

            // 4. Garder le meilleur modèle
            if (inliers_count > best_inliers_count) {
                best_inliers_count = inliers_count;
                best_slope = slope;
                best_intercept = intercept;
            }
        }

        // Vérifier si on a assez d'inliers
        float inlier_ratio = static_cast<float>(best_inliers_count) / data.size();
        if (inlier_ratio < min_inlier_ratio) {
            ESP_LOGW("RANSAC", "Ratio d'inliers trop faible: %.2f (min: %.2f)", 
                     inlier_ratio, min_inlier_ratio);
            return result;
        }

        // 5. Recalculer le modèle final avec TOUS les inliers pour plus de précision
        std::vector<std::pair<float, float>> inliers;
        for (const auto& point : data) {
            float predicted_y = best_slope * point.first + best_intercept;
            float error = std::abs(point.second - predicted_y);
            if (error <= threshold) {
                inliers.push_back(point);
            }
        }

        // Régression least squares sur les inliers
        auto final_model = least_squares(inliers);
        
        result.slope = final_model.slope;
        result.intercept = final_model.intercept;
        result.inliers_count = best_inliers_count;
        result.score = inlier_ratio;
        result.success = true;

        ESP_LOGI("RANSAC", "Modèle trouvé: slope=%.4f, intercept=%.4f, inliers=%zu/%zu (%.1f%%)",
                 result.slope, result.intercept, result.inliers_count, data.size(), inlier_ratio * 100.0f);

        return result;
    }

    /**
     * @brief Régression de Huber (M-estimateur)
     * 
     * Méthode itérative qui pénalise moins les outliers que la méthode des moindres carrés.
     * Bon compromis: plus robuste que least squares, plus précis que RANSAC.
     * Recommandé pour 10-30% d'outliers.
     * 
     * @param data Vecteur de paires (x, y)
     * @param delta Seuil de transition entre régime quadratique et linéaire (recommandé: 1.0-2.0)
     * @param max_iterations Nombre max d'itérations (recommandé: 20-50)
     * @param tolerance Critère de convergence (recommandé: 1e-4)
     * @return RegressionResult avec le modèle optimal
     */
    static RegressionResult huber(
        const std::vector<std::pair<float, float>>& data,
        float delta = 1.5f,
        int max_iterations = 30,
        float tolerance = 1e-4f)
    {
        RegressionResult result = {0.0f, 0.0f, 0.0f, 0, false};
        
        if (data.size() < 3) {
            ESP_LOGE("HUBER", "Pas assez de données (besoin d'au moins 3 points)");
            return result;
        }

        // 1. Initialisation avec régression least squares classique
        auto initial = least_squares(data);
        float slope = initial.slope;
        float intercept = initial.intercept;

        // 2. Itérations de Huber
        for (int iter = 0; iter < max_iterations; iter++) {
            float prev_slope = slope;
            float prev_intercept = intercept;

            // Calculer les résidus
            std::vector<float> residuals;
            for (const auto& point : data) {
                float predicted_y = slope * point.first + intercept;
                float residual = point.second - predicted_y;
                residuals.push_back(residual);
            }

            // Calculer les poids de Huber
            std::vector<float> weights;
            for (float r : residuals) {
                float abs_r = std::abs(r);
                float w = (abs_r <= delta) ? 1.0f : (delta / abs_r);
                weights.push_back(w);
            }

            // Régression pondérée (Weighted Least Squares)
            float sum_w = 0.0f;
            float sum_wx = 0.0f;
            float sum_wy = 0.0f;
            float sum_wxx = 0.0f;
            float sum_wxy = 0.0f;

            for (size_t i = 0; i < data.size(); i++) {
                float w = weights[i];
                float x = data[i].first;
                float y = data[i].second;

                sum_w += w;
                sum_wx += w * x;
                sum_wy += w * y;
                sum_wxx += w * x * x;
                sum_wxy += w * x * y;
            }

            // Calculer les nouveaux paramètres
            float denominator = sum_w * sum_wxx - sum_wx * sum_wx;
            if (std::abs(denominator) < 1e-10f) {
                ESP_LOGE("HUBER", "Matrice singulière à l'itération %d", iter);
                break;
            }

            slope = (sum_w * sum_wxy - sum_wx * sum_wy) / denominator;
            intercept = (sum_wy * sum_wxx - sum_wx * sum_wxy) / denominator;

            // Vérifier la convergence
            float slope_change = std::abs(slope - prev_slope);
            float intercept_change = std::abs(intercept - prev_intercept);

            if (slope_change < tolerance && intercept_change < tolerance) {
                ESP_LOGI("HUBER", "Convergence atteinte à l'itération %d", iter + 1);
                break;
            }
        }

        // Calculer le score (MAE pondéré)
        float mae = 0.0f;
        size_t inliers = 0;
        for (const auto& point : data) {
            float predicted_y = slope * point.first + intercept;
            float error = std::abs(point.second - predicted_y);
            mae += error;
            if (error <= delta) inliers++;
        }
        mae /= data.size();

        result.slope = slope;
        result.intercept = intercept;
        result.score = mae;
        result.inliers_count = inliers;
        result.success = true;

        ESP_LOGI("HUBER", "Modèle calculé: slope=%.4f, intercept=%.4f, MAE=%.4f, inliers=%zu/%zu",
                 result.slope, result.intercept, result.score, result.inliers_count, data.size());

        return result;
    }

    /**
     * @brief Régression least squares classique (pour référence/initialisation)
     */
    static RegressionResult least_squares(const std::vector<std::pair<float, float>>& data) {
        RegressionResult result = {0.0f, 0.0f, 0.0f, 0, false};
        
        if (data.size() < 2) return result;

        float sum_x = 0.0f, sum_y = 0.0f;
        for (const auto& pair : data) {
            sum_x += pair.first;
            sum_y += pair.second;
        }

        float mean_x = sum_x / data.size();
        float mean_y = sum_y / data.size();

        float variance_x = 0.0f;
        float covariance_xy = 0.0f;
        for (const auto& pair : data) {
            variance_x += (pair.first - mean_x) * (pair.first - mean_x);
            covariance_xy += (pair.first - mean_x) * (pair.second - mean_y);
        }

        if (std::abs(variance_x) < 1e-6f) return result;

        result.slope = covariance_xy / variance_x;
        result.intercept = mean_y - result.slope * mean_x;
        result.success = true;

        return result;
    }

    /**
     * @brief Méthode hybride: RANSAC pour détection outliers + Huber sur inliers
     * 
     * Combine le meilleur des deux mondes:
     * - RANSAC pour identifier et rejeter les outliers grossiers
     * - Huber pour affiner le modèle sur les données propres
     * 
     * Recommandé pour production: robustesse maximale.
     */
    static RegressionResult hybrid_ransac_huber(
        const std::vector<std::pair<float, float>>& data,
        float ransac_threshold = 1.5f,
        float huber_delta = 1.0f)
    {
        RegressionResult result = {0.0f, 0.0f, 0.0f, 0, false};
        
        // 1. Phase RANSAC pour identifier les inliers
        auto ransac_result = ransac(data, ransac_threshold, 200, 0.5f);
        if (!ransac_result.success) {
            ESP_LOGW("HYBRID", "RANSAC échoué, fallback sur Huber complet");
            return huber(data, huber_delta);
        }

        // 2. Extraire les inliers détectés par RANSAC
        std::vector<std::pair<float, float>> inliers;
        for (const auto& point : data) {
            float predicted_y = ransac_result.slope * point.first + ransac_result.intercept;
            float error = std::abs(point.second - predicted_y);
            if (error <= ransac_threshold) {
                inliers.push_back(point);
            }
        }

        ESP_LOGI("HYBRID", "RANSAC: %zu inliers identifiés sur %zu points", 
                 inliers.size(), data.size());

        // 3. Affiner avec Huber sur les inliers
        auto huber_result = huber(inliers, huber_delta, 30);
        
        if (huber_result.success) {
            result = huber_result;
            result.inliers_count = inliers.size(); // Garder le compte RANSAC
            ESP_LOGI("HYBRID", "Modèle final: slope=%.4f, intercept=%.4f", 
                     result.slope, result.intercept);
        } else {
            // Fallback sur résultat RANSAC si Huber échoue
            result = ransac_result;
            ESP_LOGW("HYBRID", "Huber échoué, utilisation du modèle RANSAC");
        }

        return result;
    }
};

} // namespace optimizer
} // namespace esphome

#endif // ROBUST_REGRESSION_H
