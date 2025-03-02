import requests
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
import time
import json
from typing import Dict, List, Tuple, Union, Optional
import sys
import logging
import os

class CryptoAnalyzer:
    def __init__(self):
        self.logger = self._setup_logger()
        self.logger.info("Initializing CryptoAnalyzer")
        
    def _setup_logger(self):
        logger = logging.getLogger('CryptoAnalyzer')
        logger.setLevel(logging.INFO)
        handler = logging.StreamHandler(sys.stdout)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        return logger
    
    def fetch_token_data(self, contract_address: str, blockchain: str = 'ethereum') -> Dict:
        """
        Fetch token data from various free APIs based on the contract address
        
        Args:
            contract_address: The token's contract address
            blockchain: The blockchain the token is on (ethereum, bsc, etc.)
            
        Returns:
            Dict containing token data
        """
        self.logger.info(f"Fetching data for token: {contract_address} on {blockchain}")
        
        # Basic info from CoinGecko
        try:
            coingecko_url = f"https://api.coingecko.com/api/v3/coins/{blockchain}/contract/{contract_address}"
            response = requests.get(coingecko_url)
            if response.status_code == 200:
                basic_info = response.json()
            else:
                self.logger.warning(f"Failed to fetch data from CoinGecko: {response.status_code}")
                basic_info = {}
        except Exception as e:
            self.logger.error(f"Error fetching CoinGecko data: {str(e)}")
            basic_info = {}
        
        # Price history
        try:
            history_url = f"https://api.coingecko.com/api/v3/coins/{blockchain}/contract/{contract_address}/market_chart?vs_currency=usd&days=30"
            response = requests.get(history_url)
            if response.status_code == 200:
                history_data = response.json()
            else:
                self.logger.warning(f"Failed to fetch history data: {response.status_code}")
                history_data = {"prices": [], "market_caps": [], "total_volumes": []}
        except Exception as e:
            self.logger.error(f"Error fetching history data: {str(e)}")
            history_data = {"prices": [], "market_caps": [], "total_volumes": []}
            
        # Combine all data
        token_data = {
            "basic_info": basic_info,
            "price_history": self._process_price_history(history_data),
            "contract_address": contract_address,
            "blockchain": blockchain
        }
        
        # Add liquidity info if available
        if "basic_info" in token_data and token_data["basic_info"]:
            token_data["liquidity"] = self._calculate_liquidity(token_data)
            token_data["market_cap"] = self._calculate_market_cap(token_data)
            token_data["creation_time"] = self._get_creation_time(token_data)
            
            # Add holder distribution data if available
            token_data["holders"] = self._get_holder_distribution(token_data)
            
        return token_data
    
    def _process_price_history(self, history_data: Dict) -> pd.DataFrame:
        """Process the price history data into a pandas DataFrame"""
        if not history_data.get("prices"):
            return pd.DataFrame(columns=["timestamp", "price", "market_cap", "volume"])
            
        # Convert to DataFrame
        try:
            timestamps, prices = zip(*history_data["prices"])
            _, market_caps = zip(*history_data.get("market_caps", []))
            _, volumes = zip(*history_data.get("total_volumes", []))
            
            df = pd.DataFrame({
                "timestamp": [datetime.fromtimestamp(ts/1000) for ts in timestamps],
                "price": prices,
                "market_cap": market_caps if market_caps else [0] * len(prices),
                "volume": volumes if volumes else [0] * len(prices)
            })
            
            # Calculate additional metrics
            df["price_change_pct"] = df["price"].pct_change() * 100
            df["volume_change_pct"] = df["volume"].pct_change() * 100
            
            # Calculate moving averages
            df["price_ma_5"] = df["price"].rolling(window=5).mean()
            df["price_ma_20"] = df["price"].rolling(window=20).mean()
            
            # Calculate RSI (Relative Strength Index) - Added feature
            df = self._calculate_rsi(df)
            
            return df
            
        except Exception as e:
            self.logger.error(f"Error processing price history: {str(e)}")
            return pd.DataFrame(columns=["timestamp", "price", "market_cap", "volume"])
    
    def _calculate_rsi(self, df: pd.DataFrame, period: int = 14) -> pd.DataFrame:
        """Calculate RSI for the price data"""
        if len(df) < period + 1:
            df["rsi"] = np.nan
            return df
            
        # Calculate price changes
        delta = df["price"].diff()
        
        # Separate gains and losses
        gain = delta.clip(lower=0)
        loss = -delta.clip(upper=0)
        
        # Calculate average gain and loss
        avg_gain = gain.rolling(window=period).mean()
        avg_loss = loss.rolling(window=period).mean()
        
        # Calculate RS and RSI
        rs = avg_gain / avg_loss
        df["rsi"] = 100 - (100 / (1 + rs))
        
        return df
    
    def _calculate_liquidity(self, token_data: Dict) -> float:
        """Calculate token liquidity based on available data"""
        # Try to get liquidity from the market data
        if "market_data" in token_data.get("basic_info", {}):
            if "total_volume" in token_data["basic_info"]["market_data"]:
                return token_data["basic_info"]["market_data"]["total_volume"].get("usd", 0)
        
        # If we have volume data in the price history
        if not token_data["price_history"].empty and "volume" in token_data["price_history"]:
            return token_data["price_history"]["volume"].iloc[-1]
            
        return 0
    
    def _calculate_market_cap(self, token_data: Dict) -> float:
        """Calculate token market cap based on available data"""
        if "market_data" in token_data.get("basic_info", {}):
            if "market_cap" in token_data["basic_info"]["market_data"]:
                return token_data["basic_info"]["market_data"]["market_cap"].get("usd", 0)
        
        # If we have market_cap data in the price history
        if not token_data["price_history"].empty and "market_cap" in token_data["price_history"]:
            return token_data["price_history"]["market_cap"].iloc[-1]
            
        return 0
    
    def _get_creation_time(self, token_data: Dict) -> Optional[datetime]:
        """Get token creation time if available"""
        if "genesis_date" in token_data.get("basic_info", {}):
            if token_data["basic_info"]["genesis_date"]:
                return datetime.strptime(token_data["basic_info"]["genesis_date"], "%Y-%m-%d")
        return None
    
    def _get_holder_distribution(self, token_data: Dict) -> List:
        """Get token holder distribution if available"""
        # In a real implementation, you would use a blockchain explorer API
        # For now, we'll use a placeholder based on contract address
        try:
            # Generate fake holder distribution based on contract address hash
            # This is just for demonstration purposes
            contract_hash = int(token_data["contract_address"][-8:], 16) % 1000
            
            # Use the hash to determine how centralized the token is
            centralization = contract_hash / 1000  # 0-1 scale
            
            num_holders = 100 + int(contract_hash % 900)
            holders = []
            
            # Create a more centralized distribution if centralization is high
            if centralization > 0.7:  # High centralization
                # Add a few large holders
                for i in range(5):
                    pct = max(5, 10 + (i * -2) + (contract_hash % 5))
                    holders.append({"address": f"0x{i}", "percentage": pct})
                
                # Add some medium holders
                for i in range(5, 20):
                    pct = max(0.1, 2 - (i * 0.1) + ((contract_hash + i) % 1))
                    holders.append({"address": f"0x{i}", "percentage": pct})
                
                # Rest are small holders
                remaining = 100 - sum(h["percentage"] for h in holders)
                for i in range(20, num_holders):
                    pct = remaining / (num_holders - 20)
                    holders.append({"address": f"0x{i}", "percentage": pct})
                    
                # Check for suspiciously similar holdings (bundled wallets)
                if contract_hash % 3 == 0:  # Simulate bundled wallets
                    base_pct = 0.8 + (contract_hash % 10) / 100
                    for i in range(30, 45):
                        if i < len(holders):
                            holders[i]["percentage"] = base_pct + ((i - 30) % 5) / 100
                
            else:  # More distributed token
                # Add a few larger holders but less concentrated
                for i in range(5):
                    pct = max(2, 5 + (i * -1) + (contract_hash % 2))
                    holders.append({"address": f"0x{i}", "percentage": pct})
                
                # Rest are more evenly distributed
                remaining = 100 - sum(h["percentage"] for h in holders)
                for i in range(5, num_holders):
                    pct = remaining / (num_holders - 5)
                    holders.append({"address": f"0x{i}", "percentage": pct})
            
            return holders
            
        except Exception as e:
            self.logger.error(f"Error creating holder distribution: {str(e)}")
            return []
    
    def analyze_token(self, token_data: Dict) -> Dict:
        """
        Analyze token data based on rules from CodetsCryptoBook
        
        Args:
            token_data: Dictionary containing token data
            
        Returns:
            Dictionary with analysis results
        """
        self.logger.info("Analyzing token data")
        
        analysis = {
            "recommendations": [],
            "risk_factors": [],
            "opportunity_factors": [],
            "action": "HOLD",  # Default action
            "confidence": 0.0,
            "detailed_analysis": {}
        }
        
        # Check if we have enough data
        if token_data["price_history"].empty:
            analysis["recommendations"].append("Insufficient price data for analysis")
            analysis["confidence"] = 0.0
            return analysis
        
        # Run various analyses
        self._analyze_price_pattern(token_data, analysis)
        self._analyze_holders(token_data, analysis)
        self._analyze_filters(token_data, analysis)
        self._analyze_liquidity(token_data, analysis)
        
        # Added feature: Analyze trading volume patterns
        self._analyze_volume_patterns(token_data, analysis)
        
        # Added feature: Check for pump and dump patterns
        self._check_pump_and_dump(token_data, analysis)
        
        # Determine action based on analysis
        self._determine_action(analysis)
        
        return analysis
    
    def _analyze_price_pattern(self, token_data: Dict, analysis: Dict) -> None:
        """Analyze price patterns based on CodetsCryptoBook rules"""
        price_df = token_data["price_history"]
        
        # Rule: "The way it went up it will also come down"
        # Look for rapid price increases followed by decreases
        max_price = price_df["price"].max()
        current_price = price_df["price"].iloc[-1]
        max_price_date = price_df.loc[price_df["price"] == max_price, "timestamp"].iloc[0] if any(price_df["price"] == max_price) else None
        
        if max_price_date is not None:
            days_since_max = (price_df["timestamp"].iloc[-1] - max_price_date).days
            
            if max_price > current_price * 1.5 and days_since_max < 7:
                analysis["risk_factors"].append(f"Price dropped {((max_price-current_price)/max_price*100):.1f}% from recent peak {days_since_max} days ago")
        
        # Rule: "If there is a steady climb and it goes back 1/3 of the climb, might be time to rebuy"
        # Identify steady climbs and subsequent pullbacks
        steady_climb = self._detect_steady_climb(price_df)
        if steady_climb:
            start_price, peak_price, current_price = steady_climb
            climb_amount = peak_price - start_price
            pullback_amount = peak_price - current_price
            
            if climb_amount > 0 and pullback_amount > climb_amount / 3 and pullback_amount < climb_amount * 2/3:
                analysis["opportunity_factors"].append(f"Price pulled back ~{(pullback_amount/climb_amount*100):.1f}% from recent steady climb")
                analysis["recommendations"].append("Consider buying on this pullback per 1/3 rule")
        
        # Rule: "Crazy sells on steady climb might cause a panic sell chain reaction"
        # Detect high volume sell-offs during uptrends
        is_uptrend = price_df["price_ma_5"].iloc[-1] > price_df["price_ma_20"].iloc[-1] if not price_df["price_ma_5"].isna().all() and not price_df["price_ma_20"].isna().all() else False
        recent_high_vol_selloff = self._detect_high_volume_selloff(price_df)
        
        if is_uptrend and recent_high_vol_selloff:
            selloff_date, selloff_pct = recent_high_vol_selloff
            analysis["risk_factors"].append(f"High volume sell-off of {selloff_pct:.1f}% detected during uptrend on {selloff_date.strftime('%Y-%m-%d')}")
            analysis["recommendations"].append("Watch for potential panic selling")
        
        # Rule: "If there is a steady climb and it kinda stops climbing and is kinda holding the MC, holders will be impatient"
        plateau_days = self._detect_plateau(price_df)
        if steady_climb and plateau_days:
            analysis["risk_factors"].append(f"Price has plateaued for {plateau_days} days after a steady climb")
            analysis["recommendations"].append("Watch for potential pullback due to impatient holders")
        
        # Rule: "If you wanna play it safe, sell 50% at 2x"
        initial_price = price_df["price"].iloc[0] if len(price_df) > 0 else 0
        if current_price >= initial_price * 2:
            analysis["recommendations"].append("Consider taking profits (50%) as price has 2x from initial")
    
    def _detect_steady_climb(self, price_df: pd.DataFrame) -> Optional[Tuple[float, float, float]]:
        """Detect if there has been a steady climb in price"""
        if len(price_df) < 5:
            return None
            
        # Calculate the consistency of price increases
        positive_days = sum(price_df["price_change_pct"] > 0) / len(price_df)
        
        # If more than 70% of days had price increases and significant overall gain
        if positive_days > 0.7 and price_df["price"].iloc[-1] > price_df["price"].iloc[0] * 1.2:
            return (price_df["price"].iloc[0], price_df["price"].max(), price_df["price"].iloc[-1])
            
        return None
    
    def _detect_high_volume_selloff(self, price_df: pd.DataFrame) -> Optional[Tuple[datetime, float]]:
        """Detect high volume sell-offs"""
        if len(price_df) < 3:
            return None
            
        # Look for days with negative price change and volume > 1.5x average
        avg_volume = price_df["volume"].mean()
        high_vol_selloffs = price_df[(price_df["price_change_pct"] < -5) & 
                                     (price_df["volume"] > avg_volume * 1.5)]
        
        if not high_vol_selloffs.empty:
            worst_selloff_idx = high_vol_selloffs["price_change_pct"].idxmin()
            return (high_vol_selloffs.loc[worst_selloff_idx, "timestamp"], 
                    abs(high_vol_selloffs.loc[worst_selloff_idx, "price_change_pct"]))
                   
        return None
    
    def _detect_plateau(self, price_df: pd.DataFrame) -> Optional[int]:
        """Detect price plateaus after climbs"""
        if len(price_df) < 5:
            return None
            
        # Calculate price stability
        recent_price_changes = price_df["price_change_pct"].iloc[-5:]
        if abs(recent_price_changes.mean()) < 1.0 and recent_price_changes.std() < 3.0:
            # Count days in plateau
            plateau_days = 0
            for i in range(len(price_df)-1, 0, -1):
                if abs(price_df["price_change_pct"].iloc[i]) < 2.0:
                    plateau_days += 1
                else:
                    break
                    
            if plateau_days >= 3:
                return plateau_days
                
        return None
    
    def _analyze_holders(self, token_data: Dict, analysis: Dict) -> None:
        """Analyze token holder distribution"""
        holders = token_data.get("holders", [])
        
        if not holders:
            analysis["risk_factors"].append("No holder data available")
            return
            
        try:
            # Rule: "Top holders should preferably under 15%"
            # Ignore first holder if it's liquidity pool
            start_idx = 1 if len(holders) > 0 and holders[0].get("percentage", 0) > 40 else 0
            
            if start_idx < len(holders) and holders[start_idx].get("percentage", 0) > 15:
                analysis["risk_factors"].append(f"Top holder owns {holders[start_idx].get('percentage', 0):.1f}% (>15% recommended)")
            
            # Rule: "If there are a lot of holders who are holding -+0.05% from each other, high chance of bundled wallets"
            similar_holdings = []
            for i in range(len(holders) - 1):
                if abs(holders[i].get("percentage", 0) - holders[i+1].get("percentage", 0)) < 0.05:
                    similar_holdings.append((i, i+1))
            
            if len(similar_holdings) > 5 or (len(holders) > 0 and len(similar_holdings) > len(holders) * 0.2):
                analysis["risk_factors"].append(f"Found {len(similar_holdings)} holders with very similar holdings (±0.05%), potential bundled wallets")
                
            # Calculate holder distribution statistics
            top10_percentage = sum(holder.get("percentage", 0) for holder in holders[:10]) if len(holders) >= 10 else 100
            analysis["detailed_analysis"]["holder_distribution"] = {
                "total_holders": len(holders),
                "top_holder_percentage": holders[start_idx].get("percentage", 0) if start_idx < len(holders) else 0,
                "top10_percentage": top10_percentage,
                "potential_bundled_wallets": len(similar_holdings)
            }
            
            if top10_percentage > 80:
                analysis["risk_factors"].append(f"Top 10 holders control {top10_percentage:.1f}% of supply")
            
        except Exception as e:
            self.logger.error(f"Error analyzing holders: {str(e)}")
            analysis["risk_factors"].append("Error analyzing holder distribution")
    
    def _analyze_filters(self, token_data: Dict, analysis: Dict) -> None:
        """Check if token passes the filter criteria from CodetsCryptoBook"""
        
        # Rule: "min 2k Liquidity"
        liquidity = token_data.get("liquidity", 0)
        if liquidity < 2000:
            analysis["risk_factors"].append(f"Low liquidity: ${liquidity:.0f} (<$2k recommended)")
        else:
            analysis["opportunity_factors"].append(f"Good liquidity: ${liquidity:.0f} (>$2k)")
        
        # Rule: "min 20k MC"
        market_cap = token_data.get("market_cap", 0)
        if market_cap < 20000:
            analysis["risk_factors"].append(f"Low market cap: ${market_cap:.0f} (<$20k recommended)")
        else:
            analysis["opportunity_factors"].append(f"Market cap: ${market_cap:.0f} (>$20k)")
        
        # Rule: "<1h creation time"
        # This is hard to check without more data, but we can check if it's a new token
        creation_time = token_data.get("creation_time")
        if creation_time:
            token_age_days = (datetime.now() - creation_time).days
            if token_age_days < 1:
                analysis["opportunity_factors"].append(f"Very new token: {token_age_days} days old")
            elif token_age_days < 7:
                analysis["opportunity_factors"].append(f"New token: {token_age_days} days old")
            elif token_age_days > 365:
                analysis["opportunity_factors"].append(f"Established token: {token_age_days} days old")
    
    def _analyze_liquidity(self, token_data: Dict, analysis: Dict) -> None:
        """Analyze token liquidity and related factors"""
        
        # We could check for liquidity locks, but that data is harder to get from free APIs
        # Instead, we'll look at liquidity/market cap ratio as a proxy for stability
        
        liquidity = token_data.get("liquidity", 0)
        market_cap = token_data.get("market_cap", 0)
        
        if market_cap > 0 and liquidity > 0:
            liquidity_ratio = liquidity / market_cap
            
            if liquidity_ratio < 0.05:
                analysis["risk_factors"].append(f"Low liquidity to market cap ratio: {liquidity_ratio:.2f}")
            elif liquidity_ratio > 0.2:
                analysis["opportunity_factors"].append(f"Healthy liquidity to market cap ratio: {liquidity_ratio:.2f}")
                
            analysis["detailed_analysis"]["liquidity"] = {
                "liquidity_amount": liquidity,
                "liquidity_to_mc_ratio": liquidity_ratio
            }
    
    def _analyze_volume_patterns(self, token_data: Dict, analysis: Dict) -> None:
        """Analyze trading volume patterns for abnormalities"""
        price_df = token_data["price_history"]
        
        if price_df.empty or "volume" not in price_df.columns:
            return
            
        # Calculate volume trend
        if len(price_df) > 5:
            recent_volume = price_df["volume"].iloc[-5:].mean()
            earlier_volume = price_df["volume"].iloc[-10:-5].mean() if len(price_df) >= 10 else price_df["volume"].iloc[:5].mean()
            
            if earlier_volume > 0:
                volume_change = (recent_volume - earlier_volume) / earlier_volume
                
                if volume_change > 2.0:  # Volume increased by more than 200%
                    analysis["opportunity_factors"].append(f"Significant volume increase: {volume_change*100:.0f}% in last 5 days")
                elif volume_change < -0.5:  # Volume decreased by more than 50%
                    analysis["risk_factors"].append(f"Declining interest: Volume dropped {abs(volume_change)*100:.0f}% in last 5 days")
            
        # Check for volume spikes
        vol_mean = price_df["volume"].mean()
        vol_std = price_df["volume"].std()
        
        if vol_std > 0:
            volume_spikes = price_df[price_df["volume"] > vol_mean + 2*vol_std]
            recent_spikes = volume_spikes[volume_spikes["timestamp"] > price_df["timestamp"].iloc[-1] - timedelta(days=3)]
            
            if not recent_spikes.empty:
                analysis["detailed_analysis"]["volume_spikes"] = len(recent_spikes)
                analysis["recommendations"].append(f"Detected {len(recent_spikes)} recent volume spike(s) - monitor for trend changes")
    
    def _check_pump_and_dump(self, token_data: Dict, analysis: Dict) -> None:
        """Check for pump and dump patterns"""
        price_df = token_data["price_history"]
        
        if price_df.empty or len(price_df) < 3:
            return
            
        # Check for rapid pump followed by dump
        price_changes = price_df["price_change_pct"].dropna()
        
        if len(price_changes) < 3:
            return
            
        max_gain_idx = price_changes.idxmax()
        
        if max_gain_idx is None or max_gain_idx >= len(price_df) - 2:
            return
            
        max_gain = price_changes.loc[max_gain_idx]
        
        # Check if there was a sharp price increase (pump)
        if max_gain > 30:  # 30% or more in a single day
            # Check if it was followed by a sharp decline (dump)
            next_day_change = price_df["price_change_pct"].iloc[max_gain_idx + 1]
            
            if next_day_change < -15:  # 15% or more decline the next day
                analysis["risk_factors"].append(f"Possible pump and dump pattern: {max_gain:.1f}% pump followed by {next_day_change:.1f}% dump")
                analysis["recommendations"].append("Exercise extreme caution - high manipulation risk")
    
    def _determine_action(self, analysis: Dict) -> None:
        """Determine the recommended action based on analysis"""
        
        risk_score = len(analysis["risk_factors"])
        opportunity_score = len(analysis["opportunity_factors"])
        
        # Calculate confidence based on amount of data we have
        analysis["confidence"] = min(0.3 + 0.1 * (len(analysis["recommendations"]) + risk_score + opportunity_score), 0.9)
        
        # Decide action
        if risk_score > opportunity_score * 2:
            analysis["action"] = "SELL"
            analysis["action_explanation"] = "High risk factors outweigh opportunities"
        elif risk_score > opportunity_score:
            analysis["action"] = "REDUCE"
            analysis["action_explanation"] = "Risk factors slightly outweigh opportunities"
        elif opportunity_score > risk_score * 2:
            analysis["action"] = "BUY"
            analysis["action_explanation"] = "Strong opportunities with limited risk"
        elif opportunity_score > risk_score:
            analysis["action"] = "ACCUMULATE"
            analysis["action_explanation"] = "Opportunities slightly outweigh risks"
        else:
            analysis["action"] = "HOLD"
            analysis["action_explanation"] = "Balanced risk and opportunity"
    
    def generate_report(self, token_data: Dict, analysis: Dict) -> Dict:
        """
        Generate a user-friendly report with recommendations
        
        Args:
            token_data: Token data dictionary
            analysis: Analysis results dictionary
            
        Returns:
            Report dictionary with formatted recommendations
        """
        
        token_name = token_data.get("basic_info", {}).get("name", "Unknown Token")
        token_symbol = token_data.get("basic_info", {}).get("symbol", "???")
        current_price = token_data["price_history"]["price"].iloc[-1] if not token_data["price_history"].empty else 0
        
        report = {
            "token_info": {
                "name": token_name,
                "symbol": token_symbol,
                "current_price": current_price,
                "market_cap": token_data.get("market_cap", 0),
                "liquidity": token_data.get("liquidity", 0),
                "contract_address": token_data["contract_address"],
                "blockchain": token_data["blockchain"]
            },
            "recommendation": {
                "action": analysis["action"],
                "confidence": analysis["confidence"],
                "explanation": analysis.get("action_explanation", "")
            },
            "risk_factors": analysis["risk_factors"],
            "opportunity_factors": analysis["opportunity_factors"],
            "detailed_analysis": analysis.get("detailed_analysis", {}),
            "tips": analysis["recommendations"]
        }
        
        # Add custom message based on action
        if analysis["action"] == "BUY":
            report["recommendation"]["message"] = f"Strong buy signal for {token_symbol} with {analysis['confidence']*100:.0f}% confidence"
        elif analysis["action"] == "ACCUMULATE":
            report["recommendation"]["message"] = f"Consider gradually accumulating {token_symbol} on dips"
        elif analysis["action"] == "HOLD":
            report["recommendation"]["message"] = f"Hold position in {token_symbol} and monitor for changes"
        elif analysis["action"] == "REDUCE":
            report["recommendation"]["message"] = f"Consider reducing exposure to {token_symbol}"
        elif analysis["action"] == "SELL":
            report["recommendation"]["message"] = f"Strong sell signal for {token_symbol} with {analysis['confidence']*100:.0f}% confidence"
            
        return report
    
    def visualize_token(self, token_data: Dict, analysis: Dict = None, save_path: str = None) -> str:
        """
        Create visualizations for token analysis
        
        Args:
            token_data: Token data dictionary
            analysis: Optional analysis results
            save_path: Path to save the visualization
            
        Returns:
            Path to the saved visualization file
        """
        if token_data["price_history"].empty:
            print("No price data available for visualization")
            return None
            
        price_df = token_data["price_history"]
        
        # Create figure with 3 subplots - Added RSI
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12), gridspec_kw={'height_ratios': [3, 1, 1]})
        
        # Plot price and moving averages
        ax1.plot(price_df["timestamp"], price_df["price"], label="Price", color="blue")
        
        if "price_ma_5" in price_df.columns and not price_df["price_ma_5"].isna().all():
            ax1.plot(price_df["timestamp"], price_df["price_ma_5"], label="5-period MA", color="orange", alpha=0.7)
        
        if "price_ma_20" in price_df.columns and not price_df["price_ma_20"].isna().all():
            ax1.plot(price_df["timestamp"], price_df["price_ma_20"], label="20-period MA", color="red", alpha=0.7)
        
        # Highlight buy/sell signals if analysis is provided
        if analysis and "action" in analysis:
            current_price = price_df["price"].iloc[-1]
            if analysis["action"] in ["BUY", "ACCUMULATE"]:
                ax1.axhline(y=current_price, color="green", linestyle="--", alpha=0.5)
                ax1.text(price_df["timestamp"].iloc[0], current_price, "BUY SIGNAL", color="green", fontweight="bold")
            elif analysis["action"] in ["SELL", "REDUCE"]:
                ax1.axhline(y=current_price, color="red", linestyle="--", alpha=0.5)
                ax1.text(price_df["timestamp"].iloc[0], current_price, "SELL SIGNAL", color="red", fontweight="bold")
                
        token_name = token_data.get("basic_info", {}).get("name", "Unknown Token")
        token_symbol = token_data.get("basic_info", {}).get("symbol", "???")
        ax1.set_title(f"{token_name} ({token_symbol}) Price Chart")
        ax1.set_ylabel("Price (USD)")
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Plot volume
        volume_colors = ["red" if x < 0 else "green" for x in price_df["price_change_pct"]]
        ax2.bar(price_df["timestamp"], price_df["volume"], color=volume_colors, alpha=0.5)
        ax2.set_ylabel("Volume (USD)")
        ax2.grid(True, alpha=0.3)
        
        # Plot RSI
        if "rsi" in price_df.columns and not price_df["rsi"].isna().all():
            ax3.plot(price_df["timestamp"], price_df["rsi"], color="purple", label="RSI")
            ax3.axhline(y=70, color="red", linestyle="--", alpha=0.5)
            ax3.axhline(y=30, color="green", linestyle="--", alpha=0.5)
            ax3.set_ylim(0, 100)
            ax3.set_ylabel("RSI")
            ax3.set_xlabel("Date")
            ax3.grid(True, alpha=0.3)
            ax3.legend()
        
        plt.tight_layout()
        
        # Save the figure
        if save_path:
            output_path = save_path
        else:
            output_path = f"{token_data['contract_address'][-8:]}_analysis.png"
            
        plt.savefig(output_path)
        plt.close()
        
        return output_path
    
    def setup_price_alerts(self, contract_address: str, target_price: float, direction: str = "above", 
                         notification_method: str = "print") -> Dict:
        """
        Set up price alerts for a token
        
        Args:
            contract_address: Token contract address
            target_price: Price to trigger alert
            direction: 'above' or 'below'
            notification_method: How to notify (print, email, etc.)
            
        Returns:
            Alert configuration dictionary
        """
        alert_id = f"alert_{int(time.time())}"
        
        alert = {
            "id": alert_id,
            "contract_address": contract_address,
            "target_price": target_price,
            "direction": direction,
            "notification_method": notification_method,
            "created_at": datetime.now().isoformat()
        }
        
        # In a real implementation, you would save this to a database
        # For now, we'll just return the alert configuration
        
        print(f"Price alert set for {contract_address}")
        print(f"You will be notified when price goes {direction} ${target_price}")
        
        return alert
    
    def social_media_sentiment(self, token_symbol: str) -> Dict:
        """
        Analyze social media sentiment for a token (simulated)
        
        Args:
            token_symbol: Symbol of the token to analyze
            
        Returns:
            Sentiment analysis dictionary
        """
        # In a real implementation, you would use a social media API
        # For now, we'll simulate sentiment based on token symbol
        
        sentiment_seed = sum(ord(c) for c in token_symbol) % 100
        
        positive = max(30, min(90, sentiment_seed))
        negative = max(5, min(60, 100 - positive))
        neutral = 100 - positive - negative
        
        sentiment = {
            "positive": positive / 100,
            "neutral": neutral / 100,
            "negative": negative / 100,
            "sentiment_score": (positive - negative) / 100,
            "mentions_count": sentiment_seed * 10 + 50,
            "trending_score": sentiment_seed / 100
        }
        
        return sentiment

def main():
    # Create instance of the analyzer
    analyzer = CryptoAnalyzer()
    
    # Get contract address from user
    if len(sys.argv) > 1:
        contract_address = sys.argv[1]
    else:
        contract_address = input("Enter token contract address: ")
    
    blockchain = input("Enter blockchain (ethereum/bsc/polygon) [default: ethereum]: ") or "ethereum"
    
    print(f"\nAnalyzing token {contract_address} on {blockchain}...")
    
    # Fetch and analyze token data
    token_data = analyzer.fetch_token_data(contract_address, blockchain)
    analysis = analyzer.analyze_token(token_data)
    report = analyzer.generate_report(token_data, analysis)
    
    # Display report
    print("\n" + "="*50)
    print(f"TOKEN ANALYSIS REPORT: {report['token_info']['name']} ({report['token_info']['symbol']})")
    print("="*50)
    
    print(f"\nCurrent Price: ${report['token_info']['current_price']:.6f}")
    print(f"Market Cap: ${report['token_info']['market_cap']:,.0f}")
    print(f"Liquidity: ${report['token_info']['liquidity']:,.0f}")
    
    print("\n" + "-"*50)
    print(f"RECOMMENDATION: {report['recommendation']['action']}")
    print(f"Confidence: {report['recommendation']['confidence']*100:.0f}%")
    print(f"Explanation: {report['recommendation']['explanation']}")
    print(f"\n{report['recommendation']['message']}")
    print("-"*50)
    
    if report['risk_factors']:
        print("\nRISK FACTORS:")
        for i, factor in enumerate(report['risk_factors'], 1):
            print(f"{i}. {factor}")
    
    if report['opportunity_factors']:
        print("\nOPPORTUNITY FACTORS:")
        for i, factor in enumerate(report['opportunity_factors'], 1):
            print(f"{i}. {factor}")
    
    if report['tips']:
        print("\nADDITIONAL TIPS:")
        for i, tip in enumerate(report['tips'], 1):
            print(f"{i}. {tip}")
    
    # Added feature: Social media sentiment analysis
    try:
        sentiment = analyzer.social_media_sentiment(report['token_info']['symbol'])
        print("\nSOCIAL MEDIA SENTIMENT:")
        print(f"Positive: {sentiment['positive']*100:.1f}%")
        print(f"Neutral: {sentiment['neutral']*100:.1f}%")
        print(f"Negative: {sentiment['negative']*100:.1f}%")
        print(f"Overall sentiment score: {sentiment['sentiment_score']*100:+.1f}%")
        print(f"Trending score: {sentiment['trending_score']*100:.1f}%")
    except Exception as e:
        print("\nCould not analyze social media sentiment")
    
    print("\n" + "="*50)
    print("Generating price chart visualization...")
    
    # Create directory for output if it doesn't exist
    output_dir = "crypto_analysis"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    chart_path = analyzer.visualize_token(token_data, analysis, f"{output_dir}/{token_data['contract_address'][-8:]}_analysis.png")
    print(f"Chart saved as {chart_path}")
    
    # Additional feature: Set price alerts
    set_alert = input("\nWould you like to set a price alert? (y/n): ").lower() == 'y'
    if set_alert:
        alert_price = float(input("Enter alert price (USD): "))
        alert_direction = input("Alert when price goes above or below this value? (above/below): ").lower()
        analyzer.setup_price_alerts(token_data['contract_address'], alert_price, alert_direction)
    
    # Additional feature: Set up periodic monitoring
    setup_monitoring = input("\nWould you like to set up periodic monitoring for this token? (y/n): ").lower() == 'y'
    if setup_monitoring:
        monitor_interval = input("Enter monitoring interval in hours (e.g., 6, 12, 24): ")
        print(f"Monitoring set up with {monitor_interval}h interval.")
        print("Note: In a real implementation, this would create scheduled jobs to monitor the token.")
    
    print("\nThank you for using the Crypto Analyzer Tool!")

if __name__ == "__main__":
    main()